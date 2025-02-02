#include <charging_indentifi/linear_pose_filter_2d.h>

// STL includes.
#include <algorithm>

// ROS includes.

#include <angles/angles.h>

LinearPoseFilter2D::LinearPoseFilter2D(const std::vector<float>& b, const std::vector<float>& a)
{
  setCoeff(b, a);
}

void LinearPoseFilter2D::setCoeff(const std::vector<float>& b, const std::vector<float>& a)
{
  //std::cout << "a" << std::endl << a << std::endl;
  //std::cout << "b" << std::endl << b << std::endl;
  // Check for the smallest vector and use that as the number of elements to push.
  size_t N = std::min(b.size(), a.size());;

  // Clear the existing coefficients.
  b_.resize(N);
  a_.resize(N);

  // Copy in the new coefficients. Only copy the N elements from the back.
  // Ideally this should be all of the elements.
  std::copy_backward(b.end() - N, b.end(), b_.end());
  std::copy_backward(a.end() - N, a.end(), a_.end());

  // Ensure that the sample vectors are appropriately sized. It is done in this way so if we
  // modify the coefficients but want to mainatain history we don't corrupt the past samples.
  int K = in_.size() - b_.size();
  if (K > 0)
  {
    // Remove the extra elements.
    for (int i = 0; i < K; i++)
    {
      in_.pop_front();
      out_.pop_front();
      yaw_in_.pop_front();
      yaw_out_.pop_front();
    }
  }
  else
  {
    // Add the extra elements.
    for (int i = 0; i > K; i--)
    {
      in_.push_front(originPose());
      out_.push_front(originPose());
      yaw_in_.push_front(tf2::getYaw(tf2::Quaternion(originPose().orientation.x, originPose().orientation.y, originPose().orientation.z, originPose().orientation.w)));
      yaw_out_.push_front(tf2::getYaw(tf2::Quaternion(originPose().orientation.x, originPose().orientation.y, originPose().orientation.z, originPose().orientation.w)));
    }
  }
}

geometry_msgs::msg::Pose LinearPoseFilter2D::filter(const geometry_msgs::msg::Pose& pose)
{
  // Add the new input pose to the vector of inputs.
  in_.push_back(pose);
  yaw_in_.push_back(getUnnormalizedYaw(pose, getNewestOutputYaw()));
  // Store a new output.
  out_.push_back(originPose());
  yaw_out_.push_back(0.0f);
  // Remove the oldest elements.
  in_.pop_front();
  out_.pop_front();
  yaw_in_.pop_front();
  yaw_out_.pop_front();

  // Grab the iterator to the new output.
  std::deque<geometry_msgs::msg::Pose>::iterator outn = --out_.end();
  std::deque<float>::iterator           yaw_outn = --yaw_out_.end();

  // Terminal index of vector.
  size_t N = b_.size() - 1;
  for (size_t i = 0; i < (N+1); i++)
  {
    // Sweep the inputs.
    outn->position.x += b_[i]*in_[N - i].position.x;
    outn->position.y += b_[i]*in_[N - i].position.y;
    *yaw_outn        += b_[i]*yaw_in_[N - i];

    // Skip the first index for the outputs.
    if (i > 0)
    {
      // Sweep the outputs.
      outn->position.x -= a_[i]*out_[N - i].position.x;
      outn->position.y -= a_[i]*out_[N - i].position.y;
      *yaw_outn        -= a_[i]*yaw_out_[N - i];
    }
  }

  // Store orientation.
  outn->orientation.z = sin(*yaw_outn/2.0);
  outn->orientation.w = cos(*yaw_outn/2.0);

  // Return the output.
  return *outn;
}

void LinearPoseFilter2D::reset()
{
  // Copy in the origins.
  setFilterState(originPose(), originPose());
}

void LinearPoseFilter2D::setFilterState(const geometry_msgs::msg::Pose& input_pose, const geometry_msgs::msg::Pose& output_pose)
{
  // Get output yaw.
  float yaw_out = tf2::getYaw(tf2::Quaternion(output_pose.orientation.x, output_pose.orientation.y, output_pose.orientation.z, output_pose.orientation.w));
  
  // Fill the output buffer with poses
  std::fill(out_.begin(), out_.end(), output_pose);
  std::fill(yaw_out_.begin(), yaw_out_.end(), yaw_out);

  // Fill the input buffer with poses.
  std::fill(in_.begin(), in_.end(), input_pose);
  std::fill(yaw_in_.begin(), yaw_in_.end(), getUnnormalizedYaw(input_pose, yaw_out));
}

void LinearPoseFilter2D::setFilterState(const std::vector<geometry_msgs::msg::Pose>& input_poses,
                                        const std::vector<geometry_msgs::msg::Pose>& output_poses)
{
  // Check length of new output set.
  // If it is less than or equal to the output buffer size, then copy all of it into
  // the output buffer.
  std::vector<geometry_msgs::msg::Pose>::const_iterator earliest_pose_out;
  if (output_poses.size() <= out_.size())
  {
    earliest_pose_out = output_poses.begin();
  }
  else
  {
    // If the new set is bigger than the output buffer, just copy the the newest
    // elements of the new set.
    earliest_pose_out = output_poses.end() - out_.size();
  }

  // Copy output poses.
  std::copy_backward(earliest_pose_out, output_poses.end(), out_.end());

  // Copy unnormalized output yaw.
  std::deque<float>::iterator yaw_out = yaw_out_.end();
  std::deque<float>::iterator yaw_previous = yaw_out_.end();
  std::vector<geometry_msgs::msg::Pose>::const_iterator pose_out = output_poses.end();
  while (pose_out != earliest_pose_out)
  {
    // // If this is the first time then initialize previous yaw.
    if (yaw_previous == yaw_out_.end())
    {
      // Dereference the newest output pose, convert it to a yaw and save it to
      // the vector of unnormalized yaws.
      *(--yaw_previous) = tf2::getYaw(tf2::Quaternion((--output_poses.end())->orientation.x, (--output_poses.end())->orientation.y, (--output_poses.end())->orientation.z, (--output_poses.end())->orientation.w));
      
    }

    // Convert the output pose to unnormalized yaw wrt the previous yaw and
    // increment the previous yaw to the current yaw.
    *(--yaw_out) = getUnnormalizedYaw(*(--pose_out), *(yaw_previous--));
  }

  // Check length of new input set.
  // If it is less than or equal to the input buffer size, then copy all of it into
  // the input buffer.
  std::vector<geometry_msgs::msg::Pose>::const_iterator earliest_pose_in;
  if (input_poses.size() <= in_.size())
  {
    earliest_pose_in = input_poses.begin();
  }
  else
  {
    // If the new set is bigger than the input buffer, just copy the the newest
    // elements of the new set.
    earliest_pose_in = input_poses.end() - in_.size();
  }

  // Copy input poses.
  std::copy_backward(earliest_pose_in, input_poses.end(), in_.end());

  // Copy unnormalized input yaw.
  yaw_previous = yaw_out_.end();
  std::deque<float>::iterator yaw_in = yaw_in_.end();
  std::vector<geometry_msgs::msg::Pose>::const_iterator pose_in = input_poses.end();
  while (pose_in != earliest_pose_in)
  {
    *(--yaw_in) = getUnnormalizedYaw(*(--pose_in), *(--yaw_previous));
  }
}

geometry_msgs::msg::Pose LinearPoseFilter2D::originPose()
{
  geometry_msgs::msg::Pose origin;
  origin.position.x     = 0;
  origin.position.y     = 0;
  origin.position.z     = 0;
  origin.orientation.x  = 0;
  origin.orientation.y  = 0;
  origin.orientation.z  = 0;
  origin.orientation.w  = 1;

  return origin;
}

float LinearPoseFilter2D::getUnnormalizedYaw(geometry_msgs::msg::Pose pose, float reference_yaw)
{
  // Get the normalized yaw.
  float yaw = tf2::getYaw(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
  


  // Add the normalized difference in angles to the reference.
  return reference_yaw + angles::normalize_angle(yaw - reference_yaw);
}

float LinearPoseFilter2D::getNewestOutputYaw()
{
  // If there aren't any previous yaws, return zero.
  if (yaw_out_.empty())
  {
    return 0.0f;
  }
  else
  {
    return *(--yaw_out_.end());
  }
}
