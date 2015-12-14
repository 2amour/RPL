/**
 * @file ros_topic_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "io/ros_topic_parser.h"

const std::string NAME_KEY = "topic_name";
const std::string QUEUE_SIZE_KEY = "queue_size";

RosTopicParser::RosTopicParser(ros::NodeHandle nh, std::string topic_name)
{
    nh.param<std::string>("topics/scan_topic", _topic_name, "scan"); // Parse cells_jumped parameter.
    nh.param<int>("topics/queue_size", _queue_size, 1); // Parse delta_theta parameter.
}

std::string RosTopicParser::get_topic_name(){
  return _topic_name;
}
int RosTopicParser::get_queue_size(){
  return _queue_size;
}
