/**
 * @file sensor_update_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs.
 */
#include <ros/ros.h>
#include "localization/sensor_update_strategies/correlation_sensor_update.h"
#include "localization/sensor_update_strategies/likelihood_sensor_update.h"

#ifndef SENSOR_UPDATE_PARSER_H_
#define SENSOR_UPDATE_PARSER_H_

/**
 * @brief Class that parses from the rosparam server the
 * sensor update strategy to be used.
 *
 */
class SensorUpdateParser
{
public:
  SensorUpdateParser(ros::NodeHandle nh); ///< Default Constructor
  ~SensorUpdateParser(); ///< Class Deconstructor
  SensorUpdatePtr get_sensor_update(); ///< Get parsed sensor strategy implementation.
  private:
  SensorUpdatePtr _sensor_update; ///< Parsed sensor strategy implementation.
};


#endif /* SENSOR_UPDATE_PARSER_H_ */
