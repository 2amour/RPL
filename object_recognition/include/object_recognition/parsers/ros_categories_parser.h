/**
 * @file ros_categories_parser.h
 *
 *  @date Dec 11, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include <ros/ros.h>
#include <string>
#include "object_recognition/util/category.h"

#ifndef ROS_CATEGORIES_PARSER_H_
#define ROS_CATEGORIES_PARSER_H_

/**
 * @brief 
 *
 */
class RosCategoriesParser
{
public:
  RosCategoriesParser(ros::NodeHandle nh); ///< Default Constructor
  std::vector<Category> get_categories(); ///< Categories getter
private:
  std::vector<Category> _categories; ///< Categories internal state
};


#endif /* ROS_CATEGORIES_PARSER_H_ */
