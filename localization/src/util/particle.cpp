/**
 * @file filter.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include <localization/util/particle.h>


Particle::Particle(){
  _weight = 1.0;
  _pose = geometry_msgs::PoseWithCovarianceStamped();
  _map.reset(new OccupancyMap);
}

Particle::Particle(double weight, geometry_msgs::PoseWithCovarianceStamped pose){
  _weight = weight;
  _pose = pose;
  _map.reset(new OccupancyMap);
}

double Particle::get_weight(){
  return _weight;
}

geometry_msgs::PoseWithCovarianceStamped Particle::get_pose(){
  return _pose;
}

OccupancyMapPtr Particle::get_map(){
  return _map;
}

void Particle::set_weight(double weight){
  _weight = weight;
}

void Particle::set_pose(geometry_msgs::PoseWithCovarianceStamped pose){
  _pose = pose;
}

void Particle::set_map(OccupancyMapPtr map){
  _map = map;
}
