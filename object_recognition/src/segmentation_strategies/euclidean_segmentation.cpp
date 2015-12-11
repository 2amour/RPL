/*
 * euclidean_segmentation.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/segmentation_strategies/euclidean_segmentation.h"
static const std::string NAME = "Euclidean_Segmentation";

EuclideanSegmentation::EuclideanSegmentation()
{
  euclidean_segmentation_parameters params = euclidean_segmentation_parameters();
  _segmentator.setClusterTolerance(params.tolerance);
  _segmentator.setMinClusterSize(params.cluster_min);
  _segmentator.setMaxClusterSize(params.cluster_max);
  _name = NAME;
}

EuclideanSegmentation::EuclideanSegmentation(struct euclidean_segmentation_parameters params)
{
  _segmentator.setClusterTolerance(params.tolerance);
  _segmentator.setMinClusterSize(params.cluster_min);
  _segmentator.setMaxClusterSize(params.cluster_max);
  _name = NAME;
}

EuclideanSegmentation::EuclideanSegmentation(double tolerance, int cluster_min, int cluster_max)
{
  _segmentator.setClusterTolerance(tolerance);
  _segmentator.setMinClusterSize(cluster_min);
  _segmentator.setMaxClusterSize(cluster_max);
  _name = NAME;
}

void EuclideanSegmentation::set_tolerance(double tolerance)
{
  _segmentator.setClusterTolerance(tolerance);
}

void EuclideanSegmentation::set_cluster_limits(int cluster_min, int cluster_max)
{
  _segmentator.setMinClusterSize(cluster_min);
  _segmentator.setMaxClusterSize(cluster_max);
}

void EuclideanSegmentation::extract_clusters(const PointCloud & cloud)
{
  _cloud = cloud;
  _cluster_indices.clear();
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud.makeShared());
  _segmentator.setSearchMethod(tree);
  _segmentator.setInputCloud(cloud.makeShared());
  _segmentator.extract(_cluster_indices);
}
