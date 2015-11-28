/*
 * object_recognition_controller.h
 *
 *  Created on: Nov 28, 2015
 *      Author: Sebastian Curi
 */

#include "../filtering_strategies/filter.h"
#include "../segmentation_strategies/segmentation.h"
#include "../correspondence_strategies/correspondence.h"
#include "../descriptors/spin_image.h"
#include "../types/points.h"
#include "../util/category.h"

#ifndef OBJECT_RECOGNITION_CONTROLLER_H_
#define OBJECT_RECOGNITION_CONTROLLER_H_


class ObjectRecognitionController {
private:
  std::vector<FilterPtr> _filters; ///< Array of pointers to abstract filters strategy.
  SegmentationPtr _segmentator; ///<Pointer to abstract segmentation strategy.
  DescriptorPtr _spin_image; ///<Pointer to SpinImage generator.
  std::vector<Category> _categories; ///< Array of categories
  CorrespondencePtr _correspondence;  ///< Pointer to abstract correspondence strategy.
public:
  ObjectRecognitionController() {}; ///< Default constructor.
  /**
   * Class constructor.
   * @param filters Array of pointers to implemented filters strategy.
   * @param segmentator Pointer to abstract implemented strategy.
   * @param spin_image Pointer to SpinImage generator.
   * @param correspondence Pointer to implemented correspondence strategy.
   */
  ObjectRecognitionController(std::vector<FilterPtr> filters, SegmentationPtr segmentator, DescriptorPtr spin_image, CorrespondencePtr correspondence);

  /**
   * Set sequence of filters to pre-process the image
   * @param filters vector of filters
   */
  void set_filters(std::vector<FilterPtr> filters);

  /**
   * Set segmentation strategy to cluster the filtered cloud
   * @param segmentator
   */
  void set_segmentator(SegmentationPtr segmentator);

  /**
   * Set the descriptor for the image recognition
   * @param spin_image
   */
  void set_descriptor(DescriptorPtr spin_image);

  /**
   * Set the correspondence strategy to match descriptors
   * @param correspondence
   */
  void set_correspondence(CorrespondencePtr correspondence);

  /**
   * Set vector of known models
   * @param categories
   */
  void set_models(std::vector<Category> categories);

  /**
   * Pre-process image method. Here the filters and segmentation algorithms are implemented.
   * @param point_cloud point_cloud to pre-process.
   */
  void pre_process_image(const PointCloud & point_cloud);

  /**
   * Recognize algorithm. Here the spin-image of the point_cloud is computed and the correspondence algorithm is performed.
   * @param point_cloud point cloud to be recognized
   * @param cluster_id id of marker tu publish if recognized!
   */
  void recognize_image(const PointCloud & point_cloud, int cluster_id);

  /**
   * Get the clusters obtained after segmentation.
   * @return a vector of PointClouds
   */
  std::vector<PointCloud> get_clusters();

  /**
   * Return the categories.
   * @return
   */
  std::vector<Category> get_categories();
};


#endif /* OBJECT_RECOGNITION_CONTROLLER_H_ */
