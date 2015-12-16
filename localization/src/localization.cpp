#include <ros/ros.h>
#include <string>

#include "localization/parsers/ros_parameter_parser.h"
#include "localization/parsers/ros_topic_parser.h"
#include "localization/behaviour/localization_behaviour.h"

static const std::string NODE_NAME = "localization";
static const std::string ODOMETRY_LISTENER = "odometry";
static const std::string SCAN_LISTENER = "scan";
static const std::string MAP_LISTENER = "map";
static const std::string REQUEST_LISTENER = "request";

static const std::string PARTICLE_PUBLISHER = "particle_array";
static const std::string ODOMETRY_PUBLISHER = "out_odometry";
static const std::string STATUS_PUBLISHER = "is_localized";

static const int NODE_RATE = 10;

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  /****************************** PARSING *******************************************/
  RosParameterParser parser(nh);

  PoseFilterPtr filter = parser.get_filter();
  MotionUpdatePtr motion_update = parser.get_motion_update();
  SensorUpdatePtr sensor_update = parser.get_sensor_update();
  SamplingPtr sampling_strategy = parser.get_sampling_strategy();

  filter->set_motion_update_strategy(motion_update);
  filter->set_sensor_update_strategy(sensor_update);
  filter->set_sampling_strategy(sampling_strategy);

  RosTopicParser odometry_topic(nh, ODOMETRY_LISTENER);
  RosTopicParser scan_topic(nh, SCAN_LISTENER);
  RosTopicParser map_topic(nh, MAP_LISTENER);
  RosTopicParser particle_topic(nh, PARTICLE_PUBLISHER);

  RosTopicParser request_topic(nh, REQUEST_LISTENER);
  RosTopicParser odometry_pub_topic(nh, ODOMETRY_PUBLISHER);
  RosTopicParser status_topic (nh, STATUS_PUBLISHER);


  /************************ LOCALIZATION BEHAVIOUR **********************************/
  ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>(particle_topic.get_topic_name(), particle_topic.get_queue_size());
  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>(odometry_pub_topic.get_topic_name(), odometry_pub_topic.get_queue_size());
  ros::Publisher status_pub = nh.advertise<std_msgs::Bool>(status_topic.get_topic_name(), status_topic.get_queue_size());



  LocalizationBehaviour behaviour;
  behaviour.set_filter(filter);
  behaviour.set_array_publisher(pose_array_pub);
  behaviour.set_odometry_publisher(odometry_pub);
  behaviour.set_status_publisher(status_pub);

  ros::Subscriber map_sub = nh.subscribe(map_topic.get_topic_name(), map_topic.get_queue_size(), &LocalizationBehaviour::map_callback, &behaviour);
  ros::Subscriber od_sub = nh.subscribe(odometry_topic.get_topic_name(), odometry_topic.get_queue_size(), &LocalizationBehaviour::odometry_callback, &behaviour);
  ros::Subscriber scan_sub =nh.subscribe(scan_topic.get_topic_name(), scan_topic.get_queue_size(), &LocalizationBehaviour::laserscan_callback, &behaviour);
  ros::Subscriber req_sub =nh.subscribe(request_topic.get_topic_name(), request_topic.get_queue_size(), &LocalizationBehaviour::request_callback, &behaviour);




  ros::Rate loop_rate(NODE_RATE);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  filter.reset();
  motion_update.reset();
  sensor_update.reset();
  sampling_strategy.reset();
  return 0;
}
