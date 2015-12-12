/** @file pass_through_filter.h
 *  @brief Filter that removes all points outside a given range.
 *
 *  This file contains a class that implements the pcl::PassThrough
 *  filter using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include "filter.h"
#include <pcl/filters/passthrough.h>

#ifndef _PASS_THROUGH_FILTER_H_
#define _PASS_THROUGH_FILTER_H_

/**
 * @brief A data structure that contains the parameters for pcl::PassThrough.
 */
struct pass_through_filter_parameters
{
  double min; ///< min of range.
  double max; ///< max of range.
  std::string field; ///< field of PCL::PointT to filter.
  pass_through_filter_parameters() :
      min(0.0), max(1.0), field("z")
  {
  } ///<Default constructor.
};

/**
 * @brief Implementation of pcl::PassThrough Filter.
 *
 * A derived class for the strategy pattern that implements the pcl::PassThrough filter.
 */
class PassThroughFilter : public Filter
{
private:
  pcl::PassThrough<PointType> _filter; ///< @brief The filter implementation.
public:
  /**
   * Default class constructor.
   */
  PassThroughFilter();

  /**
   * Class constructor.
   * @param params a filled pass_through_filter_parameters data structure.
   */
  PassThroughFilter(struct pass_through_filter_parameters params);

  /**
   * Class constructor.
   * @param min minimum of range
   * @param max maximum of range
   * @param field field of pcl::PointT to filter.
   */
  PassThroughFilter(double min, double max, std::string field);

  /**
   * Set range of filter.
   * @param min minimum of range
   * @param max maximum of range
   */
  void set_filter_limits(double min, double max);

  /**
   * Set field of pcl::PointT to filter.
   * @param field
   */
  void set_filter_field(std::string field);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to filter.
   */
  void filter(const PointCloud::Ptr & cloud);
};

#endif /* _PASS_THROUGH_FILTER_H_ */
