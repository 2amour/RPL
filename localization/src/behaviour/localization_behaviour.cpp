/**
 * @file localization_behaviour.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/behaviour/localization_behaviour.h"

LocalizationBehaviour::LocalizationBehaviour(){
  _is_requested = false;
  _was_localized = false;
}

LocalizationBehaviour::LocalizationBehaviour(PoseFilterPtr filter){
  _filter = filter;
  _is_requested = false;
   _was_localized = false;
}

void LocalizationBehaviour::set_filter(PoseFilterPtr filter){
  _filter = filter;
}

void LocalizationBehaviour::set_array_publisher(ros::Publisher pose_array_pub){
  _pose_array_publisher = pose_array_pub;
}

void LocalizationBehaviour::set_odometry_publisher(ros::Publisher odometry_pub){
  _odometry_publisher = odometry_pub;
}

void LocalizationBehaviour::set_status_publisher(ros::Publisher status_pub){
  _status_publisher = status_pub;
}

void LocalizationBehaviour::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  _filter->set_map(msg);
}
void LocalizationBehaviour::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
  _filter->motion_update(msg);
  publish_array(_filter->get_poses(), msg->header);
  publish_estimated_tf(_filter->get_best_pose(), msg->header);
  publish_odometry(_filter->get_best_pose(), msg->header);
  publish_status(_filter->is_good());
}
void LocalizationBehaviour::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if (!_is_requested){
    _filter->sensor_update(msg);
  }
}

void LocalizationBehaviour::request_callback(const std_msgs::Bool::ConstPtr& msg){
  _is_requested = msg->data;
}

void LocalizationBehaviour::publish_array(std::vector<geometry_msgs::PoseWithCovarianceStamped> array, std_msgs::Header header){
  geometry_msgs::PoseArray msg;
  msg.header.stamp = header.stamp;
  msg.header.frame_id = _filter->get_frame();
  for (auto it = array.begin(); it < array.end(); ++it){
    msg.poses.push_back(it->pose.pose);
  }
  _pose_array_publisher.publish(msg);
}

void LocalizationBehaviour::publish_estimated_tf(geometry_msgs::PoseWithCovarianceStamped pose, std_msgs::Header header){
  tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(pose.pose.pose.position.x, pose.pose.pose.position.y, 0));
  transform.setRotation(tf::Quaternion(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w));
  transform.child_frame_id_ = header.frame_id;
  transform.frame_id_ = _filter->get_frame();
  transform.stamp_ = header.stamp;

  _transform_broadcaster.sendTransform(transform);
}

void LocalizationBehaviour::publish_odometry(geometry_msgs::PoseWithCovarianceStamped pose, std_msgs::Header header){
  nav_msgs::Odometry msg;
  msg.header = header;
  msg.pose = pose.pose;
  _odometry_publisher.publish(msg);
}

void LocalizationBehaviour::publish_status(bool is_localized){
  std_msgs::Bool msg;
  msg.data = is_localized;
  _status_publisher.publish(msg);
}
