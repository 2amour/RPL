/*
 * correlation_correspondence.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: Sebastian Curi
 */

#include "correlation_correspondence.h"

CorrelationCorrespondence::CorrelationCorrespondence()
{
  correlation_correspondence_parameters params = correlation_correspondence_parameters();
  _match_threshold = params.match_threshold;
  _min_correlation = params.min_correlation;
  _K = params.K;
  _lambda = params.lambda;
  _match_percentage = 0.0;
}

CorrelationCorrespondence::CorrelationCorrespondence(struct correlation_correspondence_parameters params)
{
  _match_threshold = params.match_threshold;
  _min_correlation = params.min_correlation;
  _lambda = params.lambda;
  _K = params.K;
  _match_percentage = 0.0;
}

CorrelationCorrespondence::CorrelationCorrespondence(double match_threshold, double min_correlation, double lambda, unsigned int K)
{
  _match_threshold = match_threshold;
  _min_correlation = min_correlation;
  _K = K;
  _lambda = lambda;
  _match_percentage = 0.0;
}

void CorrelationCorrespondence::match(DescriptorCloud model_descriptor, DescriptorCloud scene_descriptor)
{
  pcl::KdTreeFLANN<DescriptorType> kdtree;
  int match = 0;
  int temp_dist2NN;
  DescriptorType searchFeature;

  //search for 2NN and check for ratio threshold
  std::vector<int> pointIdxNKNSearch(_K);
  std::vector<float> pointNKNSquaredDistance(_K);

  //initilization for kd-tree
  kdtree.setInputCloud(model_descriptor.makeShared());

  //compare each query point with each scene point
  int numFoundMatches;
  std::priority_queue<float> correlations;
  for (int c = 0; c < scene_descriptor.size(); c++)
  {
    searchFeature = scene_descriptor.points.at(c);
    numFoundMatches = kdtree.nearestKSearch(searchFeature, _K, pointIdxNKNSearch, pointNKNSquaredDistance);

    if (numFoundMatches > 0)
    {
      //temp_dist2NN = pointNKNSquaredDistance.at(0);
      float correlation = get_correlation(searchFeature, model_descriptor.points.at(pointIdxNKNSearch.at(0))); //get_atan_similarity(*s_it, *m_it, _lambda);
      correlations.push(correlation);
    }
  }

  while (correlations.size())
  {
    if (correlations.top() > _min_correlation)
    {
      correlations.pop();
      ++match;
    }
    else
    {
      break;
    }
  }
  _match_percentage = (double)match / (double)scene_descriptor.size();
}

