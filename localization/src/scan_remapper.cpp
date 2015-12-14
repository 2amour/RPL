/**
 * @file scan_remapper.cpp
 *
 *  @date Dec 7, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include "io/ros_topic_parser.h"

static const int NODE_RATE = 10;
static const std::string NODE_NAME = "scan_remapper";
static const std::string SCAN_LISTENER = "scan";
static const std::string NEW_FRAME = "base_link";

ros::Publisher scan_publisher;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::LaserScan new_msg;
  new_msg = *msg.get();
  new_msg.header.frame_id = NEW_FRAME;
  scan_publisher.publish(new_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  RosTopicParser scan_topic(nh, SCAN_LISTENER);
  scan_publisher = nh.advertise<sensor_msgs::LaserScan>(scan_topic.get_topic_name() + "remap", scan_topic.get_queue_size());
  ros::Subscriber scan_sub = nh.subscribe(scan_topic.get_topic_name(), scan_topic.get_queue_size(),scan_callback);

  ros::Rate loop_rate(NODE_RATE);

  ros::spin();

  return 0;
}
