/**
 * @file filter_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs.
 */
#include <ros/ros.h>
#include "localization/filtering_strategies/particle_filter.h"

#ifndef FILTER_PARSER_H_
#define FILTER_PARSER_H_

/**
 * @brief Class that parses from the rosparam server the
 * ros strategy to be used.
 */
class FilterParser
{
public:
  FilterParser(ros::NodeHandle nh); ///< Default Constructor
  ~FilterParser(); ///< Class Deconstructor
  PoseFilterPtr get_filter(); ///< Get parsed filter strategy implementation.
private:
  PoseFilterPtr _filter; ///< Parsed filter strategy implementation.
};


#endif /* FILTER_PARSER_H_ */
