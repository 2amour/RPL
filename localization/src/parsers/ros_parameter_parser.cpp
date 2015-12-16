/**
 * @file ros_parameter_parser.cpp
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/parsers/ros_parameter_parser.h"

RosParameterParser::RosParameterParser(ros::NodeHandle nh) :
    _filter_parser(nh), _motion_update_parser(nh), _sensor_update_parser(nh), _sampling_parser(nh)
{
  ROS_INFO("Filter: %s", _filter_parser.get_filter()->get_name().c_str());
  ROS_INFO("Motion Model: %s", _motion_update_parser.get_motion_update()->get_name().c_str());
  ROS_INFO("Sensor Model: %s", _sensor_update_parser.get_sensor_update()->get_name().c_str());
  ROS_INFO("Sampling Strategy: %s", _sampling_parser.get_sampling_strategy()->get_name().c_str());
}

PoseFilterPtr RosParameterParser::get_filter(){
  return _filter_parser.get_filter();
}
MotionUpdatePtr RosParameterParser::get_motion_update(){
  return _motion_update_parser.get_motion_update();
}
SensorUpdatePtr RosParameterParser::get_sensor_update(){
  return _sensor_update_parser.get_sensor_update();
}
SamplingPtr RosParameterParser::get_sampling_strategy(){
  return _sampling_parser.get_sampling_strategy();
}
