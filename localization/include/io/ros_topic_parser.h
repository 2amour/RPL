/**
 * @file ros_topic_parser.h
 *
 *  @date Dec 5, 2015
 *  @author Sebastian Curi
 *  @bug No known bugs
 */

#include <ros/ros.h>
#include <string>

#ifndef ROS_TOPIC_PARSER_H_
#define ROS_TOPIC_PARSER_H_

/**
 * @brief Class that parses all topics from ROS parameter server.
 *
 */
class RosTopicParser
{
public:
  /**
   * Parser constructor
   * @param nh ros node handler
   * @param topic topic to be parsed
   */
  RosTopicParser(ros::NodeHandle nh, std::string topic);
  std::string get_topic_name(); ///< Get parsed topic name.
  int get_queue_size(); ///< Get parsed topic queue size.
private:
  std::string _topic_name; ///< Parsed topic name.
  int _queue_size; ///< Parsed topic queue size.
};

#endif /* ROS_TOPIC_PARSER_H_ */
