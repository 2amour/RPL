/** @file correspondence.h
 *  @brief Abstract class for correspondence strategy pattern.
 *
 *  This file contains a class for implementing a
 *  strategy pattern. It also defines a shared pointer
 *  to it.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include "../util/histogram_math.h"

#include <vector>
#include "../descriptors/spin_image.h"
#include "../types/points.h"


#ifndef _CORRESPONDENCE_H_
#define _CORRESPONDENCE_H_

/**
 * @brief Abstract class for correspondence strategy pattern.
 *
 * A virtual class for the strategy pattern that implements correspondence between Point clouds.
 */
class Correspondence
{
protected:
  double _match_percentage; ///< percentage of matches.
  double _match_threshold; ///< minimum percentage of matches to be considered a match.
public:
  /**
   *  A virtual member to apply the correspondence strategy.
   * @param model_descriptor point cloud of model descriptors.
   * @param scene_descriptor point cloud of scene descriptors.
   */
  virtual void match(DescriptorCloud model_descriptor, DescriptorCloud scene_descriptor) = 0;

  /**
   * Virtual destructor.
   */
  virtual ~Correspondence()
  {
  }
  ;

  /**
   * Check if model and scene have mathced.
   * @return `true' if model has matched with scene.
   */
  bool has_matched(void);

  /**
   * Get the match percentage.
   * @return the match percentage.
   */
  double get_match_percentage(void);
};

/**
 * @brief Shared pointer to Correspondence class
 */
typedef boost::shared_ptr<Correspondence> CorrespondencePtr;

#endif /* _CORRESPONDENCE_H_ */
