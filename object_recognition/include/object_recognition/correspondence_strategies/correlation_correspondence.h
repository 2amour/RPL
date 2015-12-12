/** @file correlation_correspondence.h
 *  @brief Correspondence strategy based on cross-correlation between model and scene.
 *
 *  This file contains a class that implements the cross correlation
 *  correspondence using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug No known bugs.
 */
#include "correspondence.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <queue>

#ifndef _CORRELATION_CORRESPONDENCE_H_
#define _CORRELATION_CORRESPONDENCE_H_

/**
 * @brief A data structure that contains the parameters for the cross-correlation correspondence.
 */
struct correlation_correspondence_parameters
{
  double match_threshold; ///< Percentage of matching points.
  double lambda; ///< Occlusion parameter.
  double min_correlation; ///< Min cross-correlation to be considered a match.
  unsigned int K; ///< Number of neighbours to find in matching.
  correlation_correspondence_parameters() :
      match_threshold(0.9), min_correlation(0.9), K(1), lambda(0.2)
  {
  } ///<Default constructor.
};

/**
 * @brief Implementation of CorrelationCorrespondence correspondence.
 *
 * A derived class for the strategy pattern that implements the CorrelationCorrespondence correspondence.
 */
class CorrelationCorrespondence : public Correspondence
{
private:
  double _min_correlation; ///< Min cross-correlation to be considered a match.
  double _lambda; ///< Occlusion parameter.
  unsigned int _K; ///< Number of neighbours to find in matching.
public:
  /**
   * Default class constructor.
   */
  CorrelationCorrespondence();

  /**
   * Class constructor.
   * @param params a filled correlation_correspondence_parameters data structure.
   */
  CorrelationCorrespondence(struct correlation_correspondence_parameters params);

  /**
   * Class constructor.
   * @param match_threshold Percentage of matching points.
   * @param min_correlation Min cross-correlation to be considered a match.
   * @param lambda Occlusion parameter.
   * @param K Number of neighbours to find in matching.
   */
  CorrelationCorrespondence(double match_threshold, double min_correlation, double lambda, unsigned int K);

  /**
   * Implementation of virtual method
   * @param model_descriptor point cloud of model descriptors.
   * @param scene_descriptor point cloud of scene descriptors.
   */
  void match(DescriptorCloud model_descriptor, DescriptorCloud scene_descriptor);
};

#endif /* _CORRELATION_CORRESPONDENCE_H_ */
