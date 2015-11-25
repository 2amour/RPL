/*
 * voxel_grid_filter.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */
#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter() :
    Filter()
{
  voxel_filter_parameters params = voxel_filter_parameters();
  _filter.setLeafSize(params.x, params.y, params.z);
  _name = "Voxel_Grid_Filter";
  _size = (params.x + params.y + params.z)/3.0;
}

VoxelGridFilter::VoxelGridFilter(struct voxel_filter_parameters params) :
    Filter()
{
  _filter.setLeafSize(params.x, params.y, params.z);
  _name = "Voxel_Grid_Filter";
  _size = (params.x + params.y + params.z)/3.0;
}

VoxelGridFilter::VoxelGridFilter(double size) :
    Filter()
{
  _filter.setLeafSize(size, size, size);
  _name = "Voxel_Grid_Filter";
  _size = size;
}

VoxelGridFilter::VoxelGridFilter(double x, double y, double z) :
    Filter()
{
  _filter.setLeafSize(x, y, z);
  _name = "Voxel_Grid_Filter";
  _size = (x + y + z)/3.0;
}

void VoxelGridFilter::set_size(double size)
{
  _filter.setLeafSize(size, size, size);
  _size = size;
}
void VoxelGridFilter::set_size(double x, double y, double z)
{
  _filter.setLeafSize(x, y, z);
  _size = (x + y + z)/3.0;
}

double VoxelGridFilter::get_size(void)
{
  return _size;
}

void VoxelGridFilter::filter(const PointCloud::Ptr & cloud)
{
  _filter.setInputCloud(cloud);
  _filter.filter(_cloud_filtered);
}
