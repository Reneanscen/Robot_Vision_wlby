#ifndef CHARGING_INDENTIFI_LASER_PROCESSOR_H
#define CHARGING_INDENTIFI_LASER_PROCESSOR_H

#include <unistd.h>
#include <math.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <list>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace laser_processor
{
/// A struct representing a single sample from the laser.
class params
{
  public:
  std::string publish_frame;
  float max_alignment_error;
  float vmark_angle;
  float vmark_length;
  bool debug_mode;
  float split_thresh;
  uint32_t split_min_point;
  tf2::Transform scan_to_baselink_transform;
  double vangle;
  double angle_tolerance;
  double rdp_epsilon;
  double vlength;
  double length_tolerance;
  float scan_range_min;
  float scan_range_max;
  float scan_angle_min;
  float scan_angle_max;
  float mask_threshold;
  float dist_vox;
};

class Sample
{
public:
  int   index;
  float range;
  float intensity;
  float x;
  float y;

  static Sample* Extract(int ind, const sensor_msgs::msg::LaserScan& scan, params param);
  static Sample* Extract(const sensor_msgs::msg::LaserScan& scan, const pcl::PointXYZ& point); 

private:
  Sample() {};
};

/// The comparator allowing the creation of an ordered "SampleSet"
struct CompareSample
{
  CompareSample() {}

  inline bool operator()(const Sample* a, const Sample* b) const
  {
    return (a->index <  b->index);
  }
};


/// An ordered set of Samples
class SampleSet : public std::set<Sample*, CompareSample>
{
public:
  SampleSet() {}

  ~SampleSet()
  {
    clear();
  }

  void clear();

  void appendToCloud(sensor_msgs::msg::PointCloud& cloud, int r = 0, int g = 0, int b = 0);

  //tf2::Point center();
  std_msgs::msg::Header header;
};

/// A mask for filtering out Samples based on range
class ScanMask
{
  SampleSet mask_;

  bool     filled;
  float    angle_min;
  float    angle_max;
  uint32_t size;

public:
  ScanMask() : filled(false), angle_min(0), angle_max(0), size(0) { }

  inline void clear()
  {
    mask_.clear();
    filled = false;
  }

  //void addScan(sensor_msgs::msg::LaserScan& scan);

  bool hasSample(Sample* s, float thresh);
};


typedef SampleSet* SampleSetPtr;
typedef SampleSet* SampleSetConstPtr;

class ScanProcessor
{
  std::list<SampleSetConstPtr> clusters_;
  sensor_msgs::msg::LaserScan scan_;

public:
  std::list<SampleSetConstPtr>& getClusters()
  {
    return clusters_;
  }

  ScanProcessor(const sensor_msgs::msg::LaserScan& scan, ScanMask& mask_, params param);

  ~ScanProcessor();

  void removeLessThan(uint32_t num);

  void splitConnected(float thresh);

  params param_;
};
};  // namespace laser_processor

#endif  // CHARGING_INDENTIFI_LASER_PROCESSOR_H
