/*
 * eigenimage_correspondence.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: Sebastian Curi
 */

#include "eigenimage_correspondence.h"

EigenCorrespondence::EigenCorrespondence()
{
  eigenimage_correspondence_parameters params = eigenimage_correspondence_parameters();
  _min_error = params.min_error;
  _match_threshold = params.match_threshold;
  _match_percentage = 0.0;
}
EigenCorrespondence::EigenCorrespondence(struct eigenimage_correspondence_parameters params)
{
  _min_error = params.min_error;
  _match_threshold = params.match_threshold;
  _match_percentage = 0.0;
}
EigenCorrespondence::EigenCorrespondence(double min_error, double match_threshold)
{
  _min_error = min_error;
  _match_threshold = match_threshold;
  _match_percentage = 0.0;
}

void EigenCorrespondence::match(pcl::PCA<DescriptorType> model_descriptor, DescriptorCloud scene_descriptor)
{
  int match = 0;
  for (DescriptorCloud::iterator it = scene_descriptor.begin(); it != scene_descriptor.end(); ++it)
  {
    DescriptorType projected, reconstructed;
    //model_descriptor.project(*it, projected);
    //model_descriptor.reconstruct(projected, reconstructed);
    if (get_rms_error(*it, reconstructed) > _min_error)
    {
      ++match;
    }
  }
  _match_percentage = match / (double)scene_descriptor.size();
}

