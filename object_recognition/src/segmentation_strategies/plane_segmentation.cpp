/*
 * plane_segmentation.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */
#include "object_recognition/segmentation_strategies/plane_segmentation.h"
static const std::string NAME = "Plane_Segmentation";

PlaneSegmentation::PlaneSegmentation()
{
  plane_segmentation_parameters params = plane_segmentation_parameters();
  _segmentator.setOptimizeCoefficients(true);
  _segmentator.setModelType(params.model);
  _segmentator.setMethodType(params.method);
  _segmentator.setDistanceThreshold(params.distance_threshold);
  _segmentator.setMaxIterations(params.max_iter);
  _name = NAME;
}
PlaneSegmentation::PlaneSegmentation(struct plane_segmentation_parameters params)
{
  _segmentator.setOptimizeCoefficients(true);
  _segmentator.setModelType(params.model);
  _segmentator.setMethodType(params.method);
  _segmentator.setDistanceThreshold(params.distance_threshold);
  _segmentator.setMaxIterations(params.max_iter);
  _name = NAME;
}

PlaneSegmentation::PlaneSegmentation(double distance_threshold)
{
  _segmentator.setOptimizeCoefficients(true);
  _segmentator.setModelType(pcl::SACMODEL_PLANE);
  _segmentator.setMethodType(pcl::SAC_RANSAC);
  _segmentator.setDistanceThreshold(distance_threshold);
  _segmentator.setMaxIterations(100);
  _name = NAME;
}
PlaneSegmentation::PlaneSegmentation(double distance_threshold, int model, int method, int max_iter)
{
  _segmentator.setOptimizeCoefficients(true);
  _segmentator.setModelType(model);
  _segmentator.setMethodType(method);
  _segmentator.setDistanceThreshold(distance_threshold);
  _segmentator.setMaxIterations(max_iter);
  _name = NAME;
}

void PlaneSegmentation::set_distance_threshold(double distance_threshold)
{
  _segmentator.setDistanceThreshold(distance_threshold);
}
void PlaneSegmentation::set_model_type(int model)
{
  _segmentator.setModelType(model);
}
void PlaneSegmentation::set_method_type(int method)
{
  _segmentator.setMethodType(method);
}
void PlaneSegmentation::set_max_iter(int max_iter)
{
  _segmentator.setMaxIterations(max_iter);
}
void PlaneSegmentation::set_optimization(bool opt)
{
  _segmentator.setOptimizeCoefficients(opt);
}
void PlaneSegmentation::extract_clusters(const PointCloud & cloud)
{
  _cloud = cloud;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  _segmentator.setInputCloud(cloud.makeShared());
  _segmentator.segment(inliers, coefficients);
  _cluster_indices.push_back(inliers);
}
