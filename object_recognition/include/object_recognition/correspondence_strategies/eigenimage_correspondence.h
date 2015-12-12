/** @file eigenimage_correspondence.h
 *  @brief Correspondence strategy based on eigen_image projection between model and scene.
 *
 *  This file contains a class that implements the eigen_image projection
 *  correspondence using a strategy design pattern.
 *
 *  @author Sebastian Curi
 *  @date 23 Nov, 2015
 *  @bug Not implemented.
 */

#include <pcl/common/pca.h>
#include "correspondence.h"
#ifndef _EIGENIMAGE_CORRESPONDENCE_H_
#define _EIGENIMAGE_CORRESPONDENCE_H_

/**
 * @brief A data structure that contains the parameters for the eigen_image correspondence.
 */
struct eigenimage_correspondence_parameters
{
  double min_error; ///< Min error between projection and original image.
  double match_threshold; ///< Percentage of matching points.
  eigenimage_correspondence_parameters() :
      min_error(0.7), match_threshold(0.7)
  {
  }
};

/**
 * @brief Implementation of EigenCorrespondence correspondence.
 *
 * A derived class for the strategy pattern that implements the EigenCorrespondence correspondence.
 */
class EigenCorrespondence : Correspondence
{
private:
  double _min_error; ///< Min error between projection and original image.
public:
  /**
   * Default class constructor.
   */
  EigenCorrespondence();

  /**
   * Class constructor.
   * @param params a filled eigenimage_correspondence_parameters data structure.
   */
  EigenCorrespondence(struct eigenimage_correspondence_parameters params);

  /**
   * Class constructor.
   * @param min_error Min error between projection and original image.
   * @param match_threshold Percentage of matching points.
   */
  EigenCorrespondence(double min_error, double match_threshold);

  /**
   * Implementation of virtual method
   * @param model_descriptor point cloud of model descriptors.
   * @param scene_descriptor point cloud of scene descriptors.
   */
  void match(pcl::PCA<DescriptorType> model_descriptor, DescriptorCloud scene_descriptor);
};

#endif /* _EIGENIMAGE_CORRESPONDENCE_H_ */
