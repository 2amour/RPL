/** @file extract_filter.h
 *  @brief Filter that extracts points of a cloud given a index.
 *
 *  This file contains a class that implements the pcl::ExtractIndices
 *  filter using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include "filter.h"
#include <pcl/filters/extract_indices.h>

#ifndef _EXTRACT_FILTER_H_
#define _EXTRACT_FILTER_H_

/**
 * @brief Implementation of pcl::ExtractIndices Filter.
 *
 * A derived class for the strategy pattern that implements the pcl::ExtractIndices filter.
 */
class ExtractFilter : public Filter
{
private:
  pcl::ExtractIndices<PointType> _filter; ///< @brief The filter implementation.
public:

  /**
   * Default class constructor.
   */
  ExtractFilter();

  /**
   * Class Constructor.
   * @param inliers list of indices to extract.
   * @param inverse extract indices or complementary set.
   */
  ExtractFilter(pcl::PointIndices inliers, bool inverse);

  /**
   * Set the inliers to the filter.
   * @param inliers list of indices to extract.
   */
  void set_inliers(pcl::PointIndices inliers);

  /**
   * Set if filter will extract indices or complementary set.
   * @param inverse when `true' then it will extract the complementary set.
   */
  void set_inverse(bool inverse);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to filter.
   */
  void filter(const PointCloud::Ptr & cloud);
};

#endif /* _EXTRACT_FILTER_H_ */
