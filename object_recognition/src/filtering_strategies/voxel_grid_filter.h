/** @file voxel_grid_filter.h
 *  @brief Filter that downsamples the point cloud.
 *
 *  This file contains a class that implements the pcl::VoxelGrid
 *  filter using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include "filter.h"
#include <pcl/filters/voxel_grid.h>

#ifndef _VOXEL_GRID_FILTER_H_
#define _VOXEL_GRID_FILTER_H_

/**
 * @brief A data structure that contains the parameters for pcl::VoxelGrid.
 */
struct voxel_filter_parameters
{
  double x; ///< size of x downsampling vortex.
  double y; ///< size of y downsampling vortex.
  double z; ///< size of z downsampling vortex.
  voxel_filter_parameters() :
      x(0.001f), y(0.001f), z(0.001f)
  {
  } ///< Default Constructor.
  voxel_filter_parameters(double size) :
      x(size), y(size), z(size)
  {
  } ///< Constructor with a size parameter equal in all coordinates.
};

/**
 * @brief Implementation of pcl::VoxelGrid Filter.
 *
 * A derived class for the strategy pattern that implements the pcl::VoxelGrid filter.
 */
class VoxelGridFilter : public Filter
{
private:
  pcl::VoxelGrid<PointType> _filter; ///< @brief The filter implementation.
   double _size; ///< @brief size of voxel grid
public:
  /**
   * Default class constructor.
   */
  VoxelGridFilter();

  /**
   * Class constructor.
   * @param params a filled statistical_filter_parameters data structure.
   */
  VoxelGridFilter(struct voxel_filter_parameters params);

  /**
   * Class constructor.
   * @param size size downsampling vortex.
   */
  VoxelGridFilter(double size);

  /**
   * Class constructor.
   * @param x size of x downsampling vortex.
   * @param y size of y downsampling vortex.
   * @param z size of z downsampling vortex.
   */
  VoxelGridFilter(double x, double y, double z);

  /**
   * Set size of downsampling vortex.
   * @param size size of downsampling vortex.
   */
  void set_size(double size);

  /**
   * Set size of downsampling vortex.
   * @param x size of x downsampling vortex.
   * @param y size of y downsampling vortex.
   * @param z size of z downsampling vortex.
   */
  void set_size(double x, double y, double z);

  /**
   * Get size of voxel grid
   * @return voxel grid mean size.
   */
  double get_size(void);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to filter.
   */
  void filter(const PointCloud::Ptr & cloud);
};

#endif /* _VOXEL_GRID_FILTER_H_ */
