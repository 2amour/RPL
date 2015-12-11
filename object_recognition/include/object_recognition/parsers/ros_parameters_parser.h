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

static const std::string IMAGE_FRAME_PARAM_KEY = "/camera/driver/depth_frame_id"; ///< @brief image camera frame

static const std::string CORRELATION_CORRESPONDENCE_PARAM_KEY = "correlation"; ///< @brief Key for file with correlation correspondence parameters.
static const std::string EIGENIMAGE_CORRESPONDENCE_PARAM_KEY = "eigen_image"; ///< @brief Key for file with eigen image correspondence parameters.
static const std::string SPIN_IMAGE_PARAM_KEY = "spin_image"; ///< @brief Key for file with spin image parameters.
static const std::string PASS_THROUGH_PARAM_KEY = "pass_through_filter"; ///< @brief Key for file with pass-through filter parameters.
static const std::string STATISTICAL_PARAM_KEY = "statistical_filter"; ///< @brief Key for file with statistical filter parameters.
static const std::string VOXEL_PARAM_KEY = "voxel_grid_filter"; ///< @brief Key for file with voxel grid parameters.
static const std::string EUCLIDEAN_SEGMENTATION_PARAM_KEY = "euclidean_segmentation"; ///< @brief Key for file with euclidean segmentation parameters.
static const std::string PLANE_SEGMENTATION_PARAM_KEY = "plane_segmentation";///< @brief Key for file with plane segmentation parameters.
static const std::string REGION_GROWING_PARAM_KEY = "region_growing_segmentation"; ///< @brief Key for file with region growing segmentation parameters.

static const std::string MIN_CORRELATION_KEY = "min_correlation"; ///< @brief Key for min_correlation parameter
static const std::string MATCH_THRESHOLD_KEY = "match_threshold"; ///< @brief Key for match_threshold percentage parameter
static const std::string K_NEIGHBOURS_KEY = "K"; ///< @brief Key for lambda parameter
static const std::string LAMBDA_KEY = "lambda"; ///< @brief Key for lambda parameter

static const std::string FIELD_KEY = "field"; ///< @brief Key for field parameter
static const std::string MIN_KEY = "min"; ///< @brief Key for min parameter
static const std::string MAX_KEY = "max"; ///< @brief Key for max parameter
static const std::string POINTS_KEY = "points"; ///< @brief Key for points parameter
static const std::string STD_DEV_KEY = "std_dev"; ///< @brief Key for std_dev_multiplier parameter
static const std::string SIZE_KEY = "size"; ///< @brief Key for size parameter
static const std::string X_KEY = "x"; ///< @brief Key for x size parameter
static const std::string Y_KEY = "y"; ///< @brief Key for y size parameter
static const std::string Z_KEY = "z";  ///< @brief Key for z size parameter

static const std::string CLUSTER_MIN_KEY = "cluster_min"; ///<@brief Key to min number of clusters.
static const std::string CLUSTER_MAX_KEY = "cluster_max"; ///<@brief Key to max number of clusters.
static const std::string TOLERANCE_KEY = "tolerance"; ///<@brief Key to tolerance between of clusters.
static const std::string DISTANCE_THRESHOLD_KEY = "distance_threshold"; ///<@brief Key to distance threshold.
static const std::string MODEL_KEY = "model"; ///<@brief Key to plane model.
static const std::string METHOD_KEY = "method"; ///<@brief Key to plane search method.
static const std::string MAX_ITER_KEY = "max_iter"; ///<@brief Key to max number of iters in search method.
static const std::string NEIGHBOURS_KEY = "neighbours"; ///<@brief Key to min number of neighbours.
static const std::string SMOOTHNESS_THRESHOLD_KEY = "smoothness_threshold"; ///<@brief Key to smoothness threshold.
static const std::string CURVATURE_THRESHOLD_KEY = "curvature_threshold"; ///<@brief Key to curvature threshold.

static const std::string IMAGE_WIDTH_KEY = "image_width"; ///< @brief Key for image width.
static const std::string SUPPORT_ANGLE_COS_KEY = "support_angle_cos"; ///< @brief Key for cosine of angle.
static const std::string MIN_POINTS_NEIGHB_KEY =  "min_pts_neighb"; ///< @brief Key for minimum number of neighbours.
static const std::string RADIUS_KEY = "radius"; ///< @brief Key for radius.
static const std::string MAX_POINTS_KEY = "max_points"; ///< @brief Key for max_points.

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
