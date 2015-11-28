/*
 * object_recognition_controller.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition_controller.h"

ObjectRecognitionController::ObjectRecognitionController(std::vector<FilterPtr> filters, SegmentationPtr segmentator,
                                                         DescriptorPtr spin_image, CorrespondencePtr correspondence) :
    _filters(filters), _segmentator(segmentator), _spin_image(spin_image), _correspondence(correspondence)
{
}

void ObjectRecognitionController::set_filters(std::vector<FilterPtr> filters)
{
  _filters = filters;
}

void ObjectRecognitionController::set_segmentator(SegmentationPtr segmentator)
{
  _segmentator = segmentator;
}

void ObjectRecognitionController::set_descriptor(DescriptorPtr spin_image)
{
  _spin_image = spin_image;
}

void ObjectRecognitionController::set_correspondence(CorrespondencePtr correspondence)
{
  _correspondence = correspondence;
}

void ObjectRecognitionController::pre_process_image(const PointCloud & point_cloud)
{
  PointCloud pc(point_cloud);
  for (std::vector<FilterPtr>::iterator it = _filters.begin(); it != _filters.end(); ++it)
  {
    ((*it).get())->filter(pc.makeShared());
    pc = ((*it).get())->get_filtered_cloud();
  }
  _segmentator->extract_clusters(pc);
}

void ObjectRecognitionController::recognize_image(const PointCloud & point_cloud, int cluster_id)
{
  for (std::vector<Category>::iterator it = _categories.begin(); it != _categories.end(); ++it)
  {
    it->set_detected(false);
  }
  bool has_matched = false;
  try
  {
    DescriptorCloud output = _spin_image->compute_spin_image(point_cloud.makeShared());
    for (std::vector<Category>::iterator it = _categories.begin(); it != _categories.end() - 1; ++it)
    {
      bool detected = false;
      std::vector<DescriptorCloud> learning_set = it->get_learning_set();
      for (std::vector<DescriptorCloud>::iterator ls_it = learning_set.begin(); ls_it != learning_set.end(); ++ls_it)
      {
        _correspondence->match(*ls_it, output);
        detected |= _correspondence->has_matched();
        std::cout << "Cluster matched with category" << it->get_name() << "with "
            << _correspondence->get_match_percentage() << "%" << std::endl;
        it->set_detected(detected);
        has_matched |= detected;
        if (detected)
        {
          break;
        }
      }
      if (has_matched)
      {
        break;
      }
    }

  }
  catch (const pcl::PCLException& e)
  {
    std::cerr << e.what() << std::endl;
  }
  //set_unknown:
  _categories.back().set_detected(!has_matched);
}

void ObjectRecognitionController::set_models(std::vector<Category> categories)
{
  _categories = categories;
  std::cout << "Categories set!" << std::endl;
}

std::vector<PointCloud> ObjectRecognitionController::get_clusters()
{
  std::vector<PointCloud> clusters;
  for (size_t i = 0; i < (_segmentator->get_cluster_indices()).size(); ++i)
  {
    clusters.push_back(_segmentator->get_cluster_cloud(i));
  }
  return clusters;
}

std::vector<Category> ObjectRecognitionController::get_categories()
{
  return _categories;
}

