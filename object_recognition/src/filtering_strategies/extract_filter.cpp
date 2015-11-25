/*
 * extract_filter.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "extract_filter.h"

ExtractFilter::ExtractFilter() :
    Filter()
{
  _name = "Extract_Filter";
}

ExtractFilter::ExtractFilter(pcl::PointIndices inliers, bool inverse)
{
  pcl::PointIndices::Ptr p_inliers(new pcl::PointIndices(inliers));
  _filter.setIndices(p_inliers);
  _filter.setNegative(inverse);
}

void ExtractFilter::set_inliers(pcl::PointIndices inliers)
{
  pcl::PointIndices::Ptr p_inliers(new pcl::PointIndices(inliers));
  _filter.setIndices(p_inliers);

}
void ExtractFilter::set_inverse(bool inverse)
{
  _filter.setNegative(inverse);
}

void ExtractFilter::filter(const PointCloud::Ptr & cloud)
{
  _filter.setInputCloud(cloud);
  _filter.filter(_cloud_filtered);
}
