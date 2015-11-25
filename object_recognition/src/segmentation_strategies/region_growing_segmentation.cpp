/*
 * region_growing_segmentation.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "region_growing_segmentation.h"

RegionGrowingSegmentation::RegionGrowingSegmentation()
{
  region_growing_parameters params = region_growing_parameters();
  _segmentator.setMinClusterSize(params.cluster_min);
  _segmentator.setMaxClusterSize(params.cluster_max);
  _segmentator.setNumberOfNeighbours(params.neighbours);
  _segmentator.setSmoothnessThreshold(params.smoothness_threshold);
  _segmentator.setCurvatureThreshold(params.curvature_threshold);
  _name = "Region_Growing_Segmentaiton";
}

RegionGrowingSegmentation::RegionGrowingSegmentation(struct region_growing_parameters params)
{
  _segmentator.setMinClusterSize(params.cluster_min);
  _segmentator.setMaxClusterSize(params.cluster_max);
  _segmentator.setNumberOfNeighbours(params.neighbours);
  _segmentator.setSmoothnessThreshold(params.smoothness_threshold);
  _segmentator.setCurvatureThreshold(params.curvature_threshold);
  _name = "Region_Growing_Segmentaiton";
}

RegionGrowingSegmentation::RegionGrowingSegmentation(int cluster_min, int cluster_max, int neighbours,
                                                     double smoothness_threshold, float curvature_threshold)
{
  _segmentator.setMinClusterSize(cluster_min);
  _segmentator.setMaxClusterSize(cluster_max);
  _segmentator.setNumberOfNeighbours(neighbours);
  _segmentator.setSmoothnessThreshold(smoothness_threshold);
  _segmentator.setCurvatureThreshold(curvature_threshold);
  _name = "Region_Growing_Segmentaiton";
}

void RegionGrowingSegmentation::set_cluster_limits(int cluster_min, int cluster_max)
{
  _segmentator.setMinClusterSize(cluster_min);
  _segmentator.setMaxClusterSize(cluster_max);
}
void RegionGrowingSegmentation::set_number_of_neighbours(int neighbours)
{
  _segmentator.setNumberOfNeighbours(neighbours);
}
void RegionGrowingSegmentation::set_smoothness_threshold(double smoothness_threshold)
{
  _segmentator.setSmoothnessThreshold(smoothness_threshold);
}
void RegionGrowingSegmentation::set_curvature_threshold(double curvature_threshold)
{
  _segmentator.setCurvatureThreshold(curvature_threshold);
}

void RegionGrowingSegmentation::extract_clusters(const PointCloud & cloud)
{
  _cloud = cloud;
  pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> >(
      new pcl::search::KdTree<PointType>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud.makeShared());
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);
  _segmentator.setInputCloud(cloud.makeShared());
  _segmentator.setInputNormals(normals);
  _segmentator.extract(_cluster_indices);
}
