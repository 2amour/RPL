/*
 * ros_parameters_parser.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/parsers/ros_parameters_parser.h"

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

RosParameterParser::RosParameterParser(ros::NodeHandle nh)
{
  parse_filter_parameters(nh);
  parse_segmentator_parameters(nh);
  parse_descriptor_parameters(nh);
  parse_correspondence_parameters(nh);
  parse_image_frame(nh);
}

std::vector<FilterPtr> RosParameterParser::get_filters()
{
  return _filters;
}

SegmentationPtr RosParameterParser::get_segmentator()
{
  return _segmentator;
}

DescriptorPtr RosParameterParser::get_descriptor()
{
  return _descriptor;
}

CorrespondencePtr RosParameterParser::get_correspondence()
{
  return _correspondence;
}

std::string RosParameterParser::get_image_frame(){
  return _image_frame;
}

void RosParameterParser::parse_filter_parameters(ros::NodeHandle nh)
{
  if (nh.hasParam(PASS_THROUGH_PARAM_KEY))
  {
    pass_through_filter_parameters pt_filter_parameters = pass_through_filter_parameters();
    nh.getParam("/" + PASS_THROUGH_PARAM_KEY + "/" + FIELD_KEY, pt_filter_parameters.field);
    nh.getParam("/" + PASS_THROUGH_PARAM_KEY + "/" + MIN_KEY, pt_filter_parameters.min);
    nh.getParam("/" + PASS_THROUGH_PARAM_KEY + "/" + MAX_KEY, pt_filter_parameters.max);

    FilterPtr filt(new PassThroughFilter(pt_filter_parameters));
    _filters.push_back(filt);
  }
  if (nh.hasParam(STATISTICAL_PARAM_KEY))
  {
    statistical_filter_parameters st_filter_parameters = statistical_filter_parameters();
    int points = 0;
    nh.getParam("/" + STATISTICAL_PARAM_KEY + "/" + POINTS_KEY, points);
    st_filter_parameters.points = points;
    nh.getParam("/" + STATISTICAL_PARAM_KEY + "/" + STD_DEV_KEY, st_filter_parameters.std_dev_mul);

    FilterPtr filt(new StatisticalFilter(st_filter_parameters));
    _filters.push_back(filt);
  }
  if (nh.hasParam(VOXEL_PARAM_KEY))
  {
    voxel_filter_parameters v_filter_parameters = voxel_filter_parameters();
    double size;
    nh.getParam("/" + VOXEL_PARAM_KEY + "/" + SIZE_KEY, size);
    v_filter_parameters = voxel_filter_parameters(size);
    FilterPtr filt(new VoxelGridFilter(v_filter_parameters));
    _filters.push_back(filt);
  }
  if (_filters.empty())
  {
    statistical_filter_parameters s_filter_parameters = statistical_filter_parameters();
    FilterPtr filt(new StatisticalFilter(s_filter_parameters));
    _filters.push_back(filt);
    ROS_WARN("Statistical filter set by default");
  }

}

void RosParameterParser::parse_segmentator_parameters(ros::NodeHandle nh)
{
  if (nh.hasParam(EUCLIDEAN_SEGMENTATION_PARAM_KEY))
  {
    euclidean_segmentation_parameters e_segmentation_params = euclidean_segmentation_parameters();
    nh.getParam("/" + EUCLIDEAN_SEGMENTATION_PARAM_KEY + "/" + CLUSTER_MIN_KEY, e_segmentation_params.cluster_min);
    nh.getParam("/" + EUCLIDEAN_SEGMENTATION_PARAM_KEY + "/" + CLUSTER_MAX_KEY, e_segmentation_params.cluster_max);
    nh.getParam("/" + EUCLIDEAN_SEGMENTATION_PARAM_KEY + "/" + TOLERANCE_KEY, e_segmentation_params.tolerance);
    _segmentator.reset(new EuclideanSegmentation(e_segmentation_params));
  }
  if (nh.hasParam(PLANE_SEGMENTATION_PARAM_KEY))
  {
    plane_segmentation_parameters p_segmentation_params = plane_segmentation_parameters();
    nh.getParam("/" + PLANE_SEGMENTATION_PARAM_KEY + "/" + DISTANCE_THRESHOLD_KEY,
                p_segmentation_params.distance_threshold);
    nh.getParam("/" + PLANE_SEGMENTATION_PARAM_KEY + "/" + MODEL_KEY, p_segmentation_params.model);
    nh.getParam("/" + PLANE_SEGMENTATION_PARAM_KEY + "/" + METHOD_KEY, p_segmentation_params.method);
    nh.getParam("/" + PLANE_SEGMENTATION_PARAM_KEY + "/" + MAX_ITER_KEY, p_segmentation_params.max_iter);
    _segmentator.reset(new PlaneSegmentation(p_segmentation_params));
  }
  if (nh.hasParam(REGION_GROWING_PARAM_KEY))
  {
    region_growing_parameters rg_segmentation_params = region_growing_parameters();
    nh.getParam("/" + REGION_GROWING_PARAM_KEY + "/" + CLUSTER_MIN_KEY, rg_segmentation_params.cluster_min);
    nh.getParam("/" + REGION_GROWING_PARAM_KEY + "/" + CLUSTER_MAX_KEY, rg_segmentation_params.cluster_max);
    nh.getParam("/" + REGION_GROWING_PARAM_KEY + "/" + NEIGHBOURS_KEY, rg_segmentation_params.neighbours);
    nh.getParam("/" + REGION_GROWING_PARAM_KEY + "/" + SMOOTHNESS_THRESHOLD_KEY,
                rg_segmentation_params.smoothness_threshold);
    nh.getParam("/" + REGION_GROWING_PARAM_KEY + "/" + CURVATURE_THRESHOLD_KEY,
                rg_segmentation_params.curvature_threshold);
    _segmentator.reset(new RegionGrowingSegmentation(rg_segmentation_params));
  }
  if (!_segmentator)
  {
    euclidean_segmentation_parameters e_segmentation_params = euclidean_segmentation_parameters();
    _segmentator.reset(new EuclideanSegmentation(e_segmentation_params));
    ROS_WARN("Euclidean segmentation set by default");
  }

}

void RosParameterParser::parse_descriptor_parameters(ros::NodeHandle nh)
{
  if (nh.hasParam(SPIN_IMAGE_PARAM_KEY))
  {
    spin_image_params spin_params = spin_image_params();
    int value = 0;
    nh.getParam("/" + SPIN_IMAGE_PARAM_KEY + "/" + IMAGE_WIDTH_KEY, value);
    spin_params.image_width = value;
    nh.getParam("/" + SPIN_IMAGE_PARAM_KEY + "/" + SUPPORT_ANGLE_COS_KEY, spin_params.support_angle_cos);
    nh.getParam("/" + SPIN_IMAGE_PARAM_KEY + "/" + MIN_POINTS_NEIGHB_KEY, value);
    spin_params.min_pts_neighb = value;
    nh.getParam("/" + SPIN_IMAGE_PARAM_KEY + "/" + RADIUS_KEY, spin_params.radius);
    nh.getParam("/" + SPIN_IMAGE_PARAM_KEY + "/" + MAX_POINTS_KEY, value);
    spin_params.max_points = value;

    _descriptor.reset(new SpinImage(spin_params));
  }
  if (!_descriptor)
  {
    spin_image_params spin_params = spin_image_params();
    _descriptor.reset(new SpinImage(spin_params));
    ROS_WARN("Spin Image parameters set by default");
  }

}

void RosParameterParser::parse_correspondence_parameters(ros::NodeHandle nh)
{
  if (nh.hasParam(CORRELATION_CORRESPONDENCE_PARAM_KEY))
  {
    correlation_correspondence_parameters corr_corr_params = correlation_correspondence_parameters();
    nh.getParam("/" + CORRELATION_CORRESPONDENCE_PARAM_KEY + "/" + MATCH_THRESHOLD_KEY,
                corr_corr_params.match_threshold);
    nh.getParam("/" + CORRELATION_CORRESPONDENCE_PARAM_KEY + "/" + MIN_CORRELATION_KEY,
                corr_corr_params.min_correlation);
    int K = 0;
    nh.getParam("/" + CORRELATION_CORRESPONDENCE_PARAM_KEY + "/" + K_NEIGHBOURS_KEY, K);
    corr_corr_params.K = K;
    nh.getParam("/" + CORRELATION_CORRESPONDENCE_PARAM_KEY + "/" + LAMBDA_KEY, corr_corr_params.lambda);
    _correspondence.reset(new CorrelationCorrespondence(corr_corr_params));
  }

  if (!_correspondence)
  {
    correlation_correspondence_parameters corr_corr_params = correlation_correspondence_parameters();
    _correspondence.reset(new CorrelationCorrespondence(corr_corr_params));
    ROS_WARN("Correlation correspondence set by default");
  }

}

void RosParameterParser::parse_image_frame(ros::NodeHandle nh){
  nh.getParam(IMAGE_FRAME_PARAM_KEY, _image_frame);
}
