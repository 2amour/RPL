/** @file filter.h
 *  @brief Abstract class for filter strategy pattern.
 *
 *  This file contains a class for implementing a
 *  strategy pattern. It also defines a shared pointer
 *  to it.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */

#include <string.h>
#include "object_recognition/types/points.h"
#include <pcl/filters/filter.h>

#ifndef _FILTER_H_
#define _FILTER_H_

/**
 * @brief Abstract class for filter strategy pattern.
 *
 * A virtual class for the strategy pattern that implements the PCL filters.
 */
class Filter
{
protected:
  PointCloud _cloud_filtered; ///< @brief The filtered cloud.
  std::string _name; ///< @brief The name of the filter.

public:
  /**
   * A virtual member to apply the filtering strategy.
   * @param cloud The cloud to filter.
   */
  virtual void filter(const PointCloud::Ptr & cloud) = 0;

  /**
   * Virtual destructor.
   */
  virtual ~Filter()
  {
  }
  ;

  /**
   * Get the result of the filtering procedure.
   * @return The filtered cloud.
   */
  PointCloud get_filtered_cloud(void);

  /**
   * Get the name of the filter strtegy.
   * @return The filter name.
   */
  const std::string & get_name(void);
};

/**
 * @brief Shared pointer to Filter class
 */
typedef boost::shared_ptr<Filter> FilterPtr;

#endif /* _FILTER_H_ */
