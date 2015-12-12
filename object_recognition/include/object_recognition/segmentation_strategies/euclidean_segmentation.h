/** @file euclidean_segmentation.h
 *  @brief Segmentation that segments by finding points with minimum L2 norm.
 *
 *  This file contains a class that implements the pcl::EuclideanClusterExtraction
 *  segmentation using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include "segmentation.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#ifndef _EUCLIDEAN_SEGMENTATION_H_
#define _EUCLIDEAN_SEGMENTATION_H_

/**
 * @brief A data structure that contains the parameters for pcl::EuclideanClusterExtraction.
 */
struct euclidean_segmentation_parameters
{
  double tolerance; ///< maximum distance between points in cluster.
  int cluster_min; ///< minimum number of points in cluster.
  int cluster_max; ///< maximum number of points in cluster.
  euclidean_segmentation_parameters() :
      tolerance(0.02), cluster_min(100), cluster_max(2500)
  {
  } ///< Default constructor.
};

/**
 * @brief Implementation of pcl::EuclideanClusterExtraction Segmentation.
 *
 * A derived class for the strategy pattern that implements the pcl::EuclideanClusterExtraction segmentation.
 */
class EuclideanSegmentation : public Segmentation
{
private:
  pcl::EuclideanClusterExtraction<PointType> _segmentator; ///< @brief The segmentation implementation.
public:
  /**
   * Default class constructor.
   */
  EuclideanSegmentation();

  /**
   * Class constructor.
   * @param params a filled plane_segmentation_parameters data structure.
   */
  EuclideanSegmentation(struct euclidean_segmentation_parameters params);

  /**
   * Class constructor.
   * @param tolerance maximum distance between points in cluster.
   * @param cluster_min minimum number of points in cluster.
   * @param cluster_max maximum number of points in cluster.
   */
  EuclideanSegmentation(double tolerance, int cluster_min, int cluster_max);

  /**
   * Set tolerance.
   * @param tolerance maximum distance between points in cluster
   */
  void set_tolerance(double tolerance);

  /**
   * Set cluster limits.
   * @param cluster_min minimum number of points in cluster.
   * @param cluster_max maximum number of points in cluster.
   */
  void set_cluster_limits(int cluster_min, int cluster_max);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to segmented.
   */
  void extract_clusters(const PointCloud & cloud);
};

#endif /* _EUCLIDEAN_SEGMENTATION_H_ */
