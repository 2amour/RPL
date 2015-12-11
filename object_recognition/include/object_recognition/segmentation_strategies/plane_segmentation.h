/** @file plane_segmentation.h
 *  @brief Segmentation that segments by finding planes of points.
 *
 *  This file contains a class that implements the pcl::SACSegmentation
 *  segmentation using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug Not tested extensively.
 */

#include "segmentation.h"
#include <pcl/segmentation/sac_segmentation.h>

#ifndef _PLANE_SEGMENTATION_H_
#define _PLANE_SEGMENTATION_H_

/**
 * @brief A data structure that contains the parameters for pcl::SACSegmentation.
 */
struct plane_segmentation_parameters
{
  double distance_threshold; ///< minimum distance to plane.
  int model; ///< model of plane.
  int method; ///< method for finding a plane.
  int max_iter; ///< maximum number of iterations in search method.
  plane_segmentation_parameters() :
      distance_threshold(0.01), model(pcl::SACMODEL_PLANE), method(pcl::SAC_RANSAC), max_iter(100)
  {
  } ///< Default constructor.
};

/**
 * @brief Implementation of pcl::SACSegmentation Segmentation.
 *
 * A derived class for the strategy pattern that implements the pcl::SACSegmentation segmentation.
 */
class PlaneSegmentation : public Segmentation
{
private:
  pcl::SACSegmentation<PointType> _segmentator; ///< @brief The segmentation implementation.
public:
  /**
   * Default class constructor.
   */
  PlaneSegmentation();

  /**
   * Class constructor
   * @param params a filled plane_segmentation_parameters data structure.
   */
  PlaneSegmentation(struct plane_segmentation_parameters params);

  /**
   * Class constructor.
   * @param distance_threshold minimum distance to plane.
   */
  PlaneSegmentation(double distance_threshold);

  /**
   * Class Constructor
   * @param distance_threshold minimum distance to plane.
   * @param model model of plane.
   * @param method method for finding a plane.
   * @param max_iter maximum number of iterations in search method.
   */
  PlaneSegmentation(double distance_threshold, int model, int method, int max_iter);

  /**
   * Set distance threhsold.
   * @param distance_threshold minimum distance to plane.
   */
  void set_distance_threshold(double distance_threshold);

  /**
   * Set model of plane.
   * @param model model of plane.
   */
  void set_model_type(int model);

  /**
   * Set method for finding a plane.
   * @param method method for finding a plane.
   */
  void set_method_type(int method);

  /**
   * Set maximum number of iterations in search method.
   * @param max_iter maximum number of iterations in search method.
   */
  void set_max_iter(int max_iter);

  /**
   * Set optimization for search method
   * @param opt if `true' then the search is optimized.
   */
  void set_optimization(bool opt);

  /**
   * Implementation of virtual method.
   * @param cloud The cloud to segmented.
   */
  void extract_clusters(const PointCloud & cloud);
};

#endif /* _PLANE_SEGMENTATION_H_ */
