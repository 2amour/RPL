/*
 * segmentation.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "segmentation.h"

std::vector<pcl::PointIndices> Segmentation::get_cluster_indices(void)
{
  return _cluster_indices;
}

PointCloud Segmentation::get_cluster_cloud(int cluster_number)
{
  PointCloud::Ptr cloud(_cloud.makeShared());
  _filter.set_inliers(_cluster_indices.at(cluster_number));
  _filter.set_inverse(false);
  _filter.filter(cloud);
  return _filter.get_filtered_cloud();
}
PointCloud Segmentation::get_inverse_cluster_cloud(int cluster_number)
{
  PointCloud::Ptr cloud(_cloud.makeShared());
  _filter.set_inliers(_cluster_indices.at(cluster_number));
  _filter.set_inverse(true);
  _filter.filter(cloud);
  return _filter.get_filtered_cloud();
}

std::string Segmentation::get_name(void)
{
  return _name;
}
