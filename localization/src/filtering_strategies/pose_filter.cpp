/**
 * @file filter.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include <localization/filtering_strategies/pose_filter.h>


void PoseFilter::set_motion_update_strategy(MotionUpdatePtr ptr){
  _motion_update_strategy = ptr;
}

void PoseFilter::set_sensor_update_strategy(SensorUpdatePtr ptr){
  _sensor_update_strategy = ptr;
}

void PoseFilter::set_sampling_strategy(SamplingPtr ptr){
  _sampling_strategy = ptr;
}

std::string PoseFilter::get_name(){
  return _name;
}

std::string PoseFilter::get_frame(){
  return _map.header.frame_id;
}
