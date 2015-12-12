/** @file ros_parameters_parser.h
 *  @brief Parsers implementation for reading from ROS parameter server
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include "object_recognition/filtering_strategies/pass_through_filter.h"
#include "object_recognition/filtering_strategies/statistical_filter.h"
#include "object_recognition/filtering_strategies/voxel_grid_filter.h"
#include "object_recognition/segmentation_strategies/euclidean_segmentation.h"
#include "object_recognition/segmentation_strategies/plane_segmentation.h"
#include "object_recognition/segmentation_strategies/region_growing_segmentation.h"
#include "object_recognition/correspondence_strategies/correlation_correspondence.h"
#include "object_recognition/descriptors/spin_image.h"


#ifndef _ROS_PARAMETERS_PARSER_H_
#define _ROS_PARAMETERS_PARSER_H_

/**
 * @Brief Class to parse parameters from the ROS server
 */
class RosParameterParser{
private:
  std::vector<FilterPtr> _filters; ///< filter sequence
  SegmentationPtr _segmentator; ///< segmentator strategy
  DescriptorPtr _descriptor; ///< descriptor strategy
  CorrespondencePtr _correspondence; ///< correspondece strategy
  std::string _image_frame; ////< image frame

  void parse_filter_parameters(ros::NodeHandle nh); ///< parser of filters helper
  void parse_segmentator_parameters(ros::NodeHandle nh); ///< parser of segmentator helper
  void parse_descriptor_parameters(ros::NodeHandle nh); ///< parser of descriptor helper
  void parse_correspondence_parameters(ros::NodeHandle nh); ///< parser of correspondece helper
  void parse_image_frame(ros::NodeHandle nh); ///< parser of image frame helper
public:
  /**
   * Class constructor
   * @param nh ros node handler
   */
  RosParameterParser(ros::NodeHandle nh);

  /**
   * Get the parsed filters sequence
   * @return
   */
  std::vector<FilterPtr> get_filters();

  /**
   * Get the parsed segmentator
   * @return
   */
  SegmentationPtr get_segmentator();

  /**
   * Get the parsed descriptor
   * @return
   */
  DescriptorPtr get_descriptor();

  /**
   * Get the parsed correspondece
   * @return
   */
  CorrespondencePtr get_correspondence();

  /**
   * Get Image frame
   * @return the parsed image frame
   */
  std::string get_image_frame();
};



#endif /* _ROS_PARAMETERS_PARSER_H_ */
