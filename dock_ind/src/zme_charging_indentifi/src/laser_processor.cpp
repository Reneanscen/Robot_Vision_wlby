#include <charging_indentifi/laser_processor.h>
#include <stdexcept>
#include <iostream>
#include <list>
#include <set>

namespace laser_processor
{
Sample* Sample::Extract(int ind, const sensor_msgs::msg::LaserScan& scan, params param)
{
  //std::cout << "ag" << scan.angle_min + ind * scan.angle_increment << std::endl;
  if (scan.ranges[ind] > param.scan_range_min && scan.ranges[ind] < param.scan_range_max 
  && scan.angle_min + ind * scan.angle_increment > param.scan_angle_min 
  && scan.angle_min + ind * scan.angle_increment < param.scan_angle_max) //param
  {
    Sample* s = new Sample();

    s->index = ind;
    s->range = scan.ranges[ind];
    s->x = cos(scan.angle_min + ind * scan.angle_increment) * s->range;
    s->y = sin(scan.angle_min + ind * scan.angle_increment) * s->range;
    return s;
  } 
  else
  {
    return NULL;
  }
}

Sample* Sample::Extract(const sensor_msgs::msg::LaserScan& scan, const pcl::PointXYZ& point)
{
  double range = sqrt(point.x*point.x + point.y*point.y);
  float angle = atan2(point.y, point.x);
  int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
  if (range > 0.05 && range < 2)
  {
    Sample* s = new Sample();

    s->index = index;
    s->range = range;
    s->x = point.x;
    s->y = point.y;
    return s;
  } 
  else
  {
    return NULL;
  }
}


void SampleSet::clear()
{
  for (SampleSet::iterator i = begin();
       i != end();
       i++)
  {
    delete(*i);
  }
  set<Sample*, CompareSample>::clear();
}

void SampleSet::appendToCloud(sensor_msgs::msg::PointCloud& cloud, int r, int g, int b)
{
  float color_val = 0;

  int rgb = (r << 16) | (g << 8) | b;
  color_val = *reinterpret_cast<float*>(&rgb);

  for (iterator sample_iter = begin();
       sample_iter != end();
       sample_iter++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = (*sample_iter)->x;
    point.y = (*sample_iter)->y;
    point.z = 0;

    cloud.points.push_back(point);

    if (cloud.channels[0].name == "rgb")
      cloud.channels[0].values.push_back(color_val);
  }
}

/* void ScanMask::addScan(sensor_msgs::msg::LaserScan& scan)
{
  if (!filled)
  {
    angle_min = scan.angle_min;
    angle_max = scan.angle_max;
    size      = scan.ranges.size();
    filled    = true;
  }
  else if (angle_min != scan.angle_min     ||
           angle_max != scan.angle_max     ||
           size      != scan.ranges.size())
  {
    throw std::runtime_error("laser_scan::ScanMask::addScan: inconsistantly sized scans added to mask");
  }

  for (uint32_t i = 0; i < scan.ranges.size(); i++)
  {
    Sample* s = Sample::Extract(i, scan);

    if (s != NULL)
    {
      SampleSet::iterator m = mask_.find(s);

      if (m != mask_.end())
      {
        if ((*m)->range > s->range)
        {
          delete(*m);
          mask_.erase(m);
          mask_.insert(s);
        }
        else
        {
          delete s;
        }
      }
      else
      {
        mask_.insert(s);
      }
    }
  }
} */

bool ScanMask::hasSample(Sample* s, float thresh)  //thresh = 0.02 //param
{
  if (s != NULL)
  {
    SampleSet::iterator m = mask_.find(s);
    if (m != mask_.end())
      if (((*m)->range - thresh) < s->range)
      {
        //std::cout << "!11" << std::endl;
        return true;
      }
        
  }
  return false;
}

ScanProcessor::ScanProcessor(const sensor_msgs::msg::LaserScan& scan, ScanMask& mask_, params param)  //param
{
  param_ = param;
  scan_ = scan;
  SampleSet* cluster = new SampleSet;
  cluster->header = scan.header;
  for (uint32_t i = 0; i < scan.ranges.size(); i++)
  {
    Sample* s = Sample::Extract(i, scan, param_);

    if (s != NULL)
    {
      if (!mask_.hasSample(s, param_.mask_threshold))
      {
        if (cluster->size()>0)
        {

          if ((s->x - (*cluster->rbegin())->x)*(s->x - (*cluster->rbegin())->x) 
              + (s->y - (*cluster->rbegin())->y)*(s->y - (*cluster->rbegin())->y) < param_.dist_vox * param_.dist_vox) // 0.005 * 0.005 //param
          {
            delete s;
          }
          else{
            cluster->insert(s);
          }
        }
        else
        {
          cluster->insert(s);
        }
        
      }
      else
      {
        delete s;
      }
    }
  }
  clusters_.push_back(cluster);
  std::cout << "scan.ranges.size() " << scan.ranges.size() << std::endl;
}

ScanProcessor::~ScanProcessor()
{
  for (std::list<SampleSet*>::iterator c = clusters_.begin();
       c != clusters_.end();
       c++)
    delete(*c);
}

void ScanProcessor::removeLessThan(uint32_t num) //param
{
  std::list<SampleSet*>::iterator c_iter = clusters_.begin();
  while (c_iter != clusters_.end())
  {
    if ((*c_iter)->size() < num ) //param
    {
      delete(*c_iter);
      clusters_.erase(c_iter++);
    }
    else
    {
      ++c_iter;
    }
  }
  std::cout << "clusters_.size() " << clusters_.size() << std::endl;
}

void ScanProcessor::splitConnected(float thresh) //0.04
{
  std::list<SampleSet*> tmp_clusters;

  std::list<SampleSet*>::iterator c_iter = clusters_.begin();

  int i1 = 0;
  int i2 = 0;
  int i3 = 0;
  // For each cluster
  while (c_iter != clusters_.end())
  {
    // Go through the entire list
    while ((*c_iter)->size() > 0)
    {
      // Take the first element
      SampleSet::iterator s_first = (*c_iter)->begin();

      // Start a new queue
      std::list<Sample*> sample_queue;
      sample_queue.push_back(*s_first);

      (*c_iter)->erase(s_first);

      // Grow until we get to the end of the queue
      std::list<Sample*>::iterator s_q = sample_queue.begin();
      while (s_q != sample_queue.end())
      {
        int expand = static_cast<int>(asin(thresh / (*s_q)->range) / fabs(scan_.angle_increment));

        SampleSet::iterator s_rest = (*c_iter)->begin();

        //std::cout << "[" << (*s_q)->range << ", " << asin(thresh / (*s_q)->range) << ", " << scan_.angle_increment << "]" << std::endl;

        while ((s_rest != (*c_iter)->end() &&
                (*s_rest)->index < (*s_q)->index + expand))
        {
          //i1++;
          if ((*s_rest)->range - (*s_q)->range > thresh)
          {
            //i2++;
            break;
          }
          else if (sqrt(pow((*s_q)->x - (*s_rest)->x, 2.0f) + pow((*s_q)->y - (*s_rest)->y, 2.0f)) < thresh)
          {
            //i3++;
            sample_queue.push_back(*s_rest);
            (*c_iter)->erase(s_rest++);
            break;
          }
          else
          {
            ++s_rest;
          }
        }
        s_q++;
      }
      //std::cout << "sample_queue.size()" << (*sample_queue)->size() << std::endl;
      /* if (sample_queue.size() < 15)
      {
        break;
      } */

      // Move all the samples into the new cluster
      SampleSet* c = new SampleSet;
      c->header = (*c_iter)->header;
      for (s_q = sample_queue.begin(); s_q != sample_queue.end(); s_q++)
        c->insert(*s_q);

      // Store the temporary clusters
      tmp_clusters.push_back(c);
    }

    // Now that c_iter is empty, we can delete
    delete(*c_iter);

    // And remove from the map
    clusters_.erase(c_iter++);
  }

  clusters_.insert(clusters_.begin(), tmp_clusters.begin(), tmp_clusters.end());
  std::cout << "clusters_.size() " << clusters_.size() << std::endl;
  //std::cout << "i1 " << i1 << ", i2 " << i2 << ", i3 " << i3 << std::endl;
}

};  // namespace laser_processor
