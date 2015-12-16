/**
 * @file odometry_motion_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/motion_update_strategies/odometry_motion_update.h"
static const std::string NAME = "Odometry Motion Model";

OdometryMotionUpdate::OdometryMotionUpdate()
{
  _name = NAME;
  _variance.assign(4, 0.0);
  _gen.seed(rd());
}

geometry_msgs::PoseWithCovarianceStamped OdometryMotionUpdate::predict(geometry_msgs::PoseWithCovarianceStamped pose,
                                                                       const nav_msgs::Odometry::ConstPtr& old_msg,
                                                                       const nav_msgs::Odometry::ConstPtr& new_msg)
{
  /* Save pose values */
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double theta = 2*std::atan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  /* Get change in odometry */
  double theta_new = 2*std::atan2(new_msg->pose.pose.orientation.z, new_msg->pose.pose.orientation.w);
  double theta_old = 2*std::atan2(old_msg->pose.pose.orientation.z, old_msg->pose.pose.orientation.w);

  double delta_rot_1 = std::atan2(new_msg->pose.pose.position.y - old_msg->pose.pose.position.y, new_msg->pose.pose.position.x - old_msg->pose.pose.position.x) - theta_new;
  double delta_trans = std::sqrt(
      std::pow(new_msg->pose.pose.position.y - old_msg->pose.pose.position.y, 2) + std::pow(new_msg->pose.pose.position.x - old_msg->pose.pose.position.x, 2));
  double delta_rot_2 = theta_new - theta_old - delta_rot_1;

  /* Create variance distribution */
  std::normal_distribution<double> distribution_rot_1(
      0, _variance[0] * std::pow(delta_rot_1, 2) + _variance[1] * std::pow(delta_trans, 2));
  std::normal_distribution<double> distribution_trans(
      0,
      _variance[2] * std::pow(delta_trans, 2) + _variance[3] * (std::pow(delta_rot_1, 2) + std::pow(delta_rot_2, 2)));
  std::normal_distribution<double> distribution_rot_2(
      0, _variance[0] * std::pow(delta_rot_2, 2) + _variance[1] * std::pow(delta_trans, 2));

  /* Sample variances */
  double delta_rot_1_hat = delta_rot_1 + distribution_rot_1(_gen);
  double delta_trans_hat = delta_trans + distribution_trans(_gen);
  double delta_rot_2_hat = delta_rot_2 + distribution_rot_2(_gen);

  /* Update pose */

  pose.pose.pose.position.x = x + delta_trans_hat * std::cos(theta + delta_rot_1_hat);
  pose.pose.pose.position.y = y + delta_trans_hat * std::sin(theta + delta_rot_1_hat);
  theta = theta + delta_rot_1_hat + delta_rot_2_hat;
  pose.pose.pose.orientation.z = std::sin(theta/2);
  pose.pose.pose.orientation.w = std::cos(theta/2);

  return pose;
}
