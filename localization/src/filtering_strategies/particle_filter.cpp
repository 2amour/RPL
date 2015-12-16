/**
 * @file particle_filter.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/filtering_strategies/particle_filter.h"

static const std::string NAME = "Particle Filter";

ParticleFilter::ParticleFilter()
{
  _name = NAME;
  _number_of_particles = 0;
  _random_particles = 0;
  _weight_threshold = 0.5;
  _old_odometry_msg.reset(new nav_msgs::Odometry());
  _is_map_set = false;
}

ParticleFilter::ParticleFilter(int number_of_particles, int random_samples, double weight_threshold)
{
  _name = NAME;
  _number_of_particles = number_of_particles;
  _random_particles = random_samples;
  _weight_threshold = weight_threshold;
  _old_odometry_msg.reset(new nav_msgs::Odometry());
  _is_map_set = false;
  reset_particles();
}


void ParticleFilter::motion_update(const nav_msgs::Odometry::ConstPtr& new_odometry_msg)
{
  if (_old_odometry_msg->header.stamp.toSec() > 0)
  {
    for (auto it = _particles.begin(); it < _particles.end(); )
    {
      geometry_msgs::PoseWithCovarianceStamped new_pose = _motion_update_strategy->predict(it->get_pose(), _old_odometry_msg, new_odometry_msg);
      if (new_pose.pose.pose.position.x > _map.info.width * _map.info.resolution or
          new_pose.pose.pose.position.x < _map.info.origin.position.x or
          new_pose.pose.pose.position.y > _map.info.height * _map.info.resolution or
          new_pose.pose.pose.position.y < _map.info.origin.position.y)
      {
        _particles.erase(it);
      }else{
        it->set_pose(new_pose);
        ++it;
      }
    }
  }
  resample();
  _old_odometry_msg = new_odometry_msg;
}

void ParticleFilter::sensor_update(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  for (auto it = _particles.begin(); it < _particles.end(); )
  {
    double weight = _sensor_update_strategy->a_posteriori(it->get_weight(), it->get_map(), it->get_pose() , msg);
    if (weight == 0){
      _particles.erase(it);
    }else{
      it->set_weight(weight);
      ++it;
    }
  }
  resample();
}

void ParticleFilter::resample()
{
  std::vector<Particle> new_particles = _sampling_strategy->resample(_particles, _number_of_particles, _map);
  _particles = _sampling_strategy->renormalize(new_particles);
  for (int i = 0; i < _random_particles; ++i){
    _particles[i] = _sampling_strategy->random_sample(_map);
  }
  _particles = _sampling_strategy->renormalize(_particles);
}

void ParticleFilter::set_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  PoseFilter::set_map(msg);
  reset_particles();
}

std::vector<geometry_msgs::PoseWithCovarianceStamped> ParticleFilter::get_poses()
{
  std::vector<geometry_msgs::PoseWithCovarianceStamped> poses;
  for (auto it = _particles.begin(); it < _particles.end(); ++it)
  {
    poses.push_back(it->get_pose());
  }
  return poses;
}

geometry_msgs::PoseWithCovarianceStamped ParticleFilter::get_best_pose()
{
  geometry_msgs::PoseWithCovarianceStamped pose = geometry_msgs::PoseWithCovarianceStamped();
  pose.pose.pose.position.x = (_map.info.width * _map.info.resolution - _map.info.origin.position.x)/2;
  pose.pose.pose.position.y = (_map.info.height * _map.info.resolution - _map.info.origin.position.y)/2;
  pose.pose.pose.orientation.w = 1;
  double weight = 1.0/(double) _particles.size();
  for (auto it = _particles.begin(); it < _particles.end(); ++it)
  {
    if (it->get_weight() > weight)
    {
      pose = it->get_pose();
      weight = it->get_weight();
    }
  }
  return pose;
}

void ParticleFilter::set_number_of_particles(int number_of_particles)
{
  _number_of_particles = number_of_particles;
  reset_particles();
}

void ParticleFilter::set_random_particles(int random_particles)
{
  _random_particles = random_particles;
}

void ParticleFilter::reset_particles()
{
  _particles.clear();
  int sqr_particles = (int)std::sqrt(_number_of_particles);
  for (int i = 0; i < _number_of_particles; ++i)
  {
    if (_is_map_set){
      _particles.push_back(_sampling_strategy->random_sample(_map));
    }
  }
}

std::vector<Particle> ParticleFilter::get_particles()
{
  return _particles;
}

bool ParticleFilter::is_good(){
  for (auto it = _particles.begin(); it < _particles.end(); ++it)
    {
      if (it->get_weight() > _weight_threshold){
        return true;
      }
    }
  return false;
}
