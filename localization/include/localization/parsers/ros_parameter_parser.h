/**
 * @file ros_parameter_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs
 */

#include <ros/ros.h>
#include "localization/parsers/filter_parser.h"
#include "localization/parsers/motion_update_parser.h"
#include "localization/parsers/sensor_update_parser.h"
#include "localization/parsers/sampling_parser.h"

#ifndef ROS_PARAMETER_PARSER_H_
#define ROS_PARAMETER_PARSER_H_

/**
 * @brief Class that parses all parameters from ROS parameter server.
 *
 */
class RosParameterParser
{
public:
  RosParameterParser(ros::NodeHandle nh); ///< Default Constructor
  PoseFilterPtr get_filter(); ///< Get parsed filter strategy implementation.
  MotionUpdatePtr get_motion_update(); ///< Get parsed motion update strategy implementation.
  SensorUpdatePtr get_sensor_update(); ///< Get parsed sensor update strategy implementation.
  SamplingPtr get_sampling_strategy(); ///< Get parsed sampling strategy implementation.
private:
  FilterParser _filter_parser; ///< Parsed filter strategy implementation.
  MotionUpdateParser _motion_update_parser; ///< Parsed motion update strategy implementation.
  SensorUpdateParser _sensor_update_parser; ///< Parsed sensor update strategy implementation.
  SamplingParser _sampling_parser; ///< Parsed sampling strategy implementation.
};

#endif /* ROS_PARAMETER_PARSER_H_ */
