/**
 * @file odometry_tf_publisher.cpp
 *
 *  @date Dec 14, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

static const std::string NODE_NAME = "odometry_tf_publisher";
static const std::string TOPIC_NAME = "/thymio/odometry";
static const int TOPIC_QUEUE = 1;



void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  static tf::Transform transform;

  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, msg->child_frame_id));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  ros::Subscriber odometry_subscriber = nh.subscribe(TOPIC_NAME, TOPIC_QUEUE, odometry_callback);

  ros::spin();

  return 0;
}

