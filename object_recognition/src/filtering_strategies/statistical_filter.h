/** @file statistical_filter.h
 *  @brief Filter that removes all points based on statistical similarity.
 *
 *  This file contains a class that implements the pcl::StatisticalOutlierRemoval
 *  filter using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include "filter.h"
#include <pcl/filters/statistical_outlier_removal.h>

#ifndef _STATISTICAL_FILTER_H_
#define _STATISTICAL_FILTER_H_

/**
 * @brief A data structure that contains the parameters for pcl::StatisticalOutlierRemoval.
 */
struct statistical_filter_parameters
{
  unsigned int points; ///< Number of points to cluster to find statistical parameters.
  double std_dev_mul; ///< Multiplier of std_dev. All points in cluster outside the multiplier will be removed.
  statistical_filter_parameters() :
      points(50), std_dev_mul(1.0)
  {
  } ///< Default Constructor.
};

/**
 * @brief Implementation of pcl::StatisticalOutlierRemoval Filter.
 *
 * A derived class for the strategy pattern that implements the pcl::StatisticalOutlierRemoval filter.
 */
class StatisticalFilter : public Filter
{
private:
  pcl::StatisticalOutlierRemoval<PointType> _filter; ///< @brief The filter implementation.
public:
  /**
   * Default class constructor.
   */
  StatisticalFilter();

  /**
   * Class constructor.
   * @param params a filled statistical_filter_parameters data structure.
   */
  StatisticalFilter(struct statistical_filter_parameters params);

  /**
   * Class constructor.
   * @param points Number of points to cluster to find statistical parameters.
   * @param std_dev_mul Multiplier of std_dev. All points in cluster outside the multiplier will be removed.
   */
  StatisticalFilter(unsigned int points, double std_dev_mul);

  /**
   * Number point setter.
   * @param points Number of points to cluster to find statistical parameters.
   */
  void set_filter_number_of_points(unsigned int points);

  /**
   * Standard deviation multiplier setter. All points in cluster outside standard deviation times the multiplier will be removed.
   * @param std_dev_mul All points in cluster outside the multiplier will be removed.
   */
  void set_std_dev_threshold(double std_dev_mul);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to filter.
   */
  void filter(const PointCloud::Ptr & cloud);
};

#endif /* _STATISTICAL_FILTER_H_ */
