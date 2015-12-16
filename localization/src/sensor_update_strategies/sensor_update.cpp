/**
 * @file sensor_update.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sensor_update_strategies/sensor_update.h"

std::string SensorUpdate::get_name(){
  return _name;
}

std::string SensorUpdate::get_reference_frame(){
  return _map.get_reference_frame();
}

void SensorUpdate::set_map(const nav_msgs::OccupancyGrid::ConstPtr& map){
  _map.set_map(map);
}
