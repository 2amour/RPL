/** @file topics.h
 *  @brief File with topic definitions.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#ifndef _ROS_TOPICS_H_
#define _ROS_TOPICS_H_


static const std::string NODE_NAME = "object_recognition"; ///< @brief global node name
static const std::string IMAGE_TOPIC = "/camera/depth/points"; ///< @brief image topic
static const int IMAGE_QUEUE_SIZE = 1; ///< @brief image msg queue size
static const std::string MARKER_TOPIC = "/visualization_marker"; ///< @brief visualization msg topic
static const int MARKER_QUEUE_SIZE = 1; ///< @brief visualization msg queue size

#endif /* _ROS_TOPICS_H_ */
