/** @file region_growing_segmentation.h
 *  @brief Segmentation that segments by growing regions and clustering.
 *
 *  This file contains a class that implements the pcl::RegionGrowing
 *  segmentation using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug Not tested extensively.
 */

#include "segmentation.h"

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#ifndef _REGION_GROWING_SEGMENTATION_H_
#define _REGION_GROWING_SEGMENTATION_H_

/**
 * @brief A data structure that contains the parameters for pcl::RegionGrowing.
 */
struct region_growing_parameters
{
  int cluster_min; ///< min number of points for clustering.
  int cluster_max; ///< max number of points for clustering.
  int neighbours; ///< number of neighbours.
  double smoothness_threshold;  ///< smoothness maximum value.
  double curvature_threshold; ///< curvature maximum value.
  region_growing_parameters() :
      cluster_min(50), cluster_max(1000000), neighbours(30), smoothness_threshold(3.0 / 180.0 * M_PI), curvature_threshold(
          1.0)
  {
  } ///< Default Constructor.
};

/**
 * @brief Implementation of pcl::RegionGrowing Segmentation.
 *
 * A derived class for the strategy pattern that implements the pcl::RegionGrowing segmentation.
 */
class RegionGrowingSegmentation : public Segmentation
{
private:
  pcl::RegionGrowing<PointType, pcl::Normal> _segmentator; ///< @brief The segmentation implementation.
public:
  /**
   * Default class constructor.
   */
  RegionGrowingSegmentation();

  /**
   * Class constructor
   * @param params a filled region_growing_parameters data structure.
   */
  RegionGrowingSegmentation(struct region_growing_parameters params);

  /**
   * Class constructor.
   * @param cluster_min min number of points for clustering.
   * @param cluster_max max number of points for clustering.
   * @param neighbours number of neighbours.
   * @param smoothness_threshold smoothness maximum value.
   * @param curvature_threshold curvature maximum value.
   */
  RegionGrowingSegmentation(int cluster_min, int cluster_max, int neighbours, double smoothness_threshold,
                            float curvature_threshold);

  /**
   * Set cluster limits.
   * @param cluster_min min number of points for clustering.
   * @param cluster_max max number of points for clustering.
   */
  void set_cluster_limits(int cluster_min, int cluster_max);

  /**
   * Set number of neighbours.
   * @param neighbours number of neighbours.
   */
  void set_number_of_neighbours(int neighbours);

  /**
   * Set smoothness constraint parameter.
   * @param smoothness_threshold smoothness maximum value.
   */
  void set_smoothness_threshold(double smoothness_threshold);

  /**
   * Set curvature constraint parameter.
   * @param curvature_threshold curvature maximum value.
   */
  void set_curvature_threshold(double curvature_threshold);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to segmented.
   */
  void extract_clusters(const PointCloud & cloud);
};

#endif /* _REGION_GROWING_SEGMENTATION_H_ */
