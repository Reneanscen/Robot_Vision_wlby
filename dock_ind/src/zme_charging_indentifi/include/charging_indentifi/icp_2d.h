#ifndef CHARGING_INDENTIFI_ICP_2D_H
#define CHARGING_INDENTIFI_ICP_2D_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform.hpp"


#include <vector>
#include <unistd.h>
#include <iostream>

namespace icp_2d
{

/**
 * @brief Get the 2d rotation from a quaternion.
 * @param q The quaternion to convert.
 * @return The orientation in 2d.
 *
 * Note: will throw if q.x or q.y is not equal to zero
 */
double thetaFromQuaternion(const geometry_msgs::msg::Quaternion& q);

/**
 * @brief Transform a vector of points in 2d.
 * @param points The points to transform.
 * @param x The x offset to transform, in the current frame of points.
 * @param y The y offset to transform, in the current frame of points.
 * @param theta The rotation, in the current frame of points.
 * @return The transformed points.
 */
std::vector<geometry_msgs::msg::Point>
transform(const std::vector<geometry_msgs::msg::Point>& points,
          double x,
          double y,
          double theta);

/**
 * @brief Get the centroid of a set of points.
 * @param points The points to find centroid of.
 */
geometry_msgs::msg::Point
getCentroid(const std::vector<geometry_msgs::msg::Point> points);

/**
 * @brief Perform PCA algorithm to align two point clouds in
 *        a two dimensional plane.
 * @param source The cloud to be aligned with target.
 * @param target The cloud to be aligned to.
 * @param transform The transformation to align source with target.
 * @return True if successful, false otherwise.
 */
bool alignPCA(const std::vector<geometry_msgs::msg::Point> source,
              const std::vector<geometry_msgs::msg::Point> target,
              geometry_msgs::msg::Transform & transform);

/**
 * @brief Perform SVD optimization to align two point clouds
 *        in a two dimensional plane.
 * @param source The cloud to be aligned with target.
 * @param target The cloud to be aligned to.
 * @param transform The transformation to align source with target.
 * @return True if successful, false otherwise.
 */
bool alignSVD(const std::vector<geometry_msgs::msg::Point> source,
              const std::vector<geometry_msgs::msg::Point> target,
              geometry_msgs::msg::Transform & transform);

/**
 * @brief Perform Iterative Closest Point (ICP) algorithm to
 *        align two point clouds in a two dimensional plane.
 * @param source The cloud to be aligned with target.
 * @param target The cloud to be aligned to.
 * @param transform The transformation to align source with target.
 * @return Fitness score, negative if error.
 */
double alignICP(const std::vector<geometry_msgs::msg::Point> source,
                const std::vector<geometry_msgs::msg::Point> target,
                geometry_msgs::msg::Transform & transform,
                size_t max_iterations = 10,
                double min_delta_rmsd = 0.000001);

}  // namespace icp_2d

#endif  // CHARGING_INDENTIFI_ICP_2D_H
