/**
 * @file thymio_tf_launcher.cpp
 *
 * Main application file to launch the tfs between thymio, map and the camera
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

#include "parsers/tf_parameter_parser.h"
#include "publisher/tf_broadcaster.h"


static const std::string NODE_NAME = "thymio_tf_launcher";
static const std::string CAMERA_FRAME = "camera_link";
static const std::string ODOMETRY_LINK_FRAME = "odometry_link";
static const double DURATION = 1.0;

int main(int argc, char** argv){
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;
  std::vector<tf::StampedTransform> tfs;

  TfParameterParser tf_parser(nh, CAMERA_FRAME);
  tf::StampedTransform camera_tf = tf_parser.get_stamped_transform();
  tfs.push_back(camera_tf);

  tf_parser = TfParameterParser(nh, ODOMETRY_LINK_FRAME);
  tf::StampedTransform thymio_tf = tf_parser.get_stamped_transform();
  tfs.push_back(thymio_tf);

  TfBroadcaster broadcaster;
  broadcaster.set_tfs(tfs);
  ros::Timer timer = nh.createTimer(ros::Duration(DURATION), &TfBroadcaster::timerCallback, &broadcaster);
  ros::spin();
}


