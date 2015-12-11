/*
 * filter.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: Sebastian Curi
 */
#include "object_recognition/filtering_strategies/filter.h"

const std::string & Filter::get_name()
{
  return _name;
}

PointCloud Filter::get_filtered_cloud(void)
{
  return _cloud_filtered;
}

