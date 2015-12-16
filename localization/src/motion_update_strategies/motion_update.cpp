/**
 * @file motion_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/motion_update_strategies/motion_update.h"

std::string MotionUpdate::get_name(){
  return _name;
}

void MotionUpdate::set_variance(std::vector<double> variance){
  _variance = variance;
}
