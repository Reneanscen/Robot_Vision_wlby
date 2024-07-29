#ifndef CHARGING_INDENTIFI_DOCK_CANDIDATE_H
#define CHARGING_INDENTIFI_DOCK_CANDIDATE_H

#include "geometry_msgs/msg/point.hpp"

#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>

/**
 * @brief A cluster which is a candidate for a dock
 */
struct DockCandidate
{
  std::vector<geometry_msgs::msg::Point> points;
  double dist;  // distance from initial/previous pose
  double k;

  /** @brief Get the width of this segment */
  double width()
  {
    // If there are no points then there is no width.
    if (points.empty())
    {
      return 0;
    }

    geometry_msgs::msg::Point& pt1 = points.front();
    geometry_msgs::msg::Point& pt2 = points.back();
    return (sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2)));
  }

  /**
   * @brief Determine if this candidate meets our basic criteria
   * @param dock_found Has the dock been found in a previous frame?
   */
  bool valid(bool dock_found)
  {
    // If there are no points this cannot be valid.
    if (points.empty())
    {
      return false;
    }

    // Check overall size
    if (width() > 0.5 || width() < 0.25)
      return false;

    // If dock is found, we want to avoid large jumps
    if (dock_found)
    {
      // dist is squared
      return dist < (0.25*0.25);
    }
    //std::cout << "dist " << dist << std::endl;
    // Not too far off from initial pose
    return dist < 1.0;
  }
};
typedef boost::shared_ptr<DockCandidate> DockCandidatePtr;

struct CompareCandidates
{
  bool operator()(DockCandidatePtr a, DockCandidatePtr b)
  {
    return (a->dist > b->dist);
  }
};

#endif  // CHARGING_INDENTIFI_DOCK_CANDIDATE_H
