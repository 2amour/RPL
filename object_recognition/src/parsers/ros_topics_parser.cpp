/**
 * @file ros_topics_parser.cpp
 *
 *  @date Dec 11, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */


#include "object_recognition/parsers/ros_topics_parser.h"

const std::string NAME_KEY = "topic_name";
const std::string QUEUE_SIZE_KEY = "queue_size";


RosTopicParser::RosTopicParser(ros::NodeHandle nh, std::string topic_name){
  _topic_name = ""; _queue_size = 1;
  if (nh.hasParam(topic_name)){
    nh.getParam("/" + topic_name + "/" + NAME_KEY, _topic_name);
    nh.getParam("/" + topic_name + "/" + QUEUE_SIZE_KEY, _queue_size);
    ROS_INFO("Topic %s, queue size %i", _topic_name.c_str(), _queue_size);
  }else{
    ROS_WARN("Topic not found.");
  }
}

std::string RosTopicParser::get_topic_name(){
  return _topic_name;
}
int RosTopicParser::get_queue_size(){
  return _queue_size;
}


