/** @file segmentation.h
 *  @brief Abstract class for segmentation strategy pattern.
 *
 *  This file contains a class for implementing a
 *  strategy pattern. It also defines a shared pointer
 *  to it.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <vector>
#include <ros/ros.h>

#include "../types/points.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include "../filtering_strategies/extract_filter.h"

#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_

/**
 * @brief Abstract class for segmentation strategy pattern.
 *
 * A virtual class for the strategy pattern that implements the PCL segmentation.
 */
class Segmentation
{
private:
  ExtractFilter _filter; ///< @brief The filter to extract the point clouds of each cluster.
protected:
  std::vector<pcl::PointIndices> _cluster_indices;  ///< @brief A vector with each cluster index cloud.
  std::string _name; ///< @brief The name of the segmentation strategy.
  PointCloud _cloud; ///< @brief The clustered cloud.
public:
  /**
   * A virtual member to apply the segmentation strategy.
   * @param cloud The cloud to filter.
   */
  virtual void extract_clusters(const PointCloud & cloud) = 0;
  /**
   * Virtual destructor.
   */
  virtual ~Segmentation()
  {
  }
  ;

  /**
   * Get name of the segmentation strategy.
   * @return The segmentaiton name.
   */
  std::string get_name(void);

  /**
   * Get the extracted cluster of indices.
   * @return vector of indices of the different clusters.
   */
  std::vector<pcl::PointIndices> get_cluster_indices(void);

  /**
   * Get the point cloud from the cluster `cluster_number'.
   * @param cluster_number Number of cluster to extract.
   * @return Point Cloud of the cluster.
   */
  PointCloud get_cluster_cloud(int cluster_number);

  /**
   * Get the point cloud from the complement set of `cluster_number'.
   * @param cluster_number Number of cluster to extract.
   * @return Complementary Point Cloud of the cluster.
   */
  PointCloud get_inverse_cluster_cloud(int cluster_number);

};

/**
 * @brief Shared pointer to Segmentation class
 */
typedef boost::shared_ptr<Segmentation> SegmentationPtr;

#endif /* _SEGMENTATION_H_ */
