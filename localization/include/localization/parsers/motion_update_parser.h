/**
 * @file motion_update_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs.
 */
#include <ros/ros.h>
#include "localization/motion_update_strategies/odometry_motion_update.h"
#include "localization/motion_update_strategies/velocity_motion_update.h"

#ifndef MOTION_UPDATE_PARSER_H_
#define MOTION_UPDATE_PARSER_H_

/**
 * @brief Class that parses from the rosparam server the
 * motion update strategy to be used.
 *
 */
class MotionUpdateParser
{
public:
  MotionUpdateParser(ros::NodeHandle nh); ///< Default Constructor
  ~MotionUpdateParser(); ///< Class Deconstructor
  MotionUpdatePtr get_motion_update();  ///< Get parsed motion update strategy implementation.
  private:
  MotionUpdatePtr _motion_update; ///< Parsed motion update strategy implementation.
};


#endif /* MOTION_UPDATE_PARSER_H_ */
