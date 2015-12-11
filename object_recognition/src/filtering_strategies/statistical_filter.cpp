/*
 * statistical_filter.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/filtering_strategies/statistical_filter.h"
static const std::string NAME = "Statistical_Filter";

StatisticalFilter::StatisticalFilter()
{
  statistical_filter_parameters parameters = statistical_filter_parameters();
  _filter.setMeanK(parameters.points);
  _filter.setStddevMulThresh(parameters.std_dev_mul);
  _name = NAME;
}

StatisticalFilter::StatisticalFilter(struct statistical_filter_parameters parameters)
{
  _filter.setMeanK(parameters.points);
  _filter.setStddevMulThresh(parameters.std_dev_mul);
  _name = NAME;
}

StatisticalFilter::StatisticalFilter(unsigned int points, double std_dev_mul)
{
  _filter.setMeanK(points);
  _filter.setStddevMulThresh(std_dev_mul);
  _name = NAME;
}

void StatisticalFilter::set_filter_number_of_points(unsigned int points)
{
  _filter.setMeanK(points);
}

void StatisticalFilter::set_std_dev_threshold(double std_dev_mul)
{
  _filter.setStddevMulThresh(std_dev_mul);
}

void StatisticalFilter::filter(const PointCloud::Ptr & cloud)
{
  _filter.setInputCloud(cloud);
  _filter.filter(_cloud_filtered);
}
