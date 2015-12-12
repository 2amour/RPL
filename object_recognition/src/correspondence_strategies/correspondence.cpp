/*
 * correspondence.hpp
 *
 *  Created on: Nov 22, 2015
 *      Author: Sebastian Curi
 */

#include "object_recognition/correspondence_strategies/correspondence.h"

double Correspondence::get_match_percentage(void)
{
  return _match_percentage;
}

bool Correspondence::has_matched(void)
{
  return _match_percentage > _match_threshold;
}
