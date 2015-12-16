/**
 * @file velocity_motion_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/motion_update_strategies/velocity_motion_update.h"
static const std::string NAME = "Velocity Motion Model";

VelocityMotionUpdate::VelocityMotionUpdate()
{
  _name = NAME;
  _variance.assign(6, 0.0);
  _gen.seed(rd());
}

geometry_msgs::PoseWithCovarianceStamped VelocityMotionUpdate::predict(geometry_msgs::PoseWithCovarianceStamped pose,
                                                                       const nav_msgs::Odometry::ConstPtr& old_msg,
                                                                       const nav_msgs::Odometry::ConstPtr& new_msg)
{
    /* Save pose values */
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double theta = 2*std::atan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);

  /* Get velocities */
  double dt = new_msg->header.stamp.toSec() - old_msg->header.stamp.toSec();
  double v = new_msg->twist.twist.linear.x;
  double w = new_msg->twist.twist.angular.z;

  /* Create variance distribution */
  std::normal_distribution<double> distribution_v(0, _variance[0] * std::pow(v, 2) + _variance[1] * std::pow(w, 2));
  std::normal_distribution<double> distribution_w(0, _variance[2] * std::pow(v, 2) + _variance[3] * std::pow(w, 2));
  std::normal_distribution<double> distribution_theta(0, _variance[4] * std::pow(v, 2) + _variance[5] * std::pow(w, 2));

  /* Sample variances */
  double v_hat = v + distribution_v(_gen);
  double w_hat = w + distribution_w(_gen);
  double theta_hat = distribution_theta(_gen);

  /* Update pose */
  if (w_hat == 0){
    pose.pose.pose.position.x = x + v_hat * dt * std::cos(theta);
    pose.pose.pose.position.y = y + v_hat * dt * std::sin(theta);
  }else{
    pose.pose.pose.position.x = x - v_hat / w_hat * std::sin(theta) + v_hat / w_hat * std::sin(theta + w_hat * dt);
    pose.pose.pose.position.y = y + v_hat / w_hat * std::cos(theta) - v_hat / w_hat * std::cos(theta + w_hat * dt);
  }
  theta += (w_hat + theta_hat)*dt;
  pose.pose.pose.orientation.z = std::sin(theta/2);
  pose.pose.pose.orientation.w = std::cos(theta/2);
  return pose;
}

