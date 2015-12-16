/**
 * @file sampling.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sampling_strategies/sampling.h"

std::string Sampling::get_name()
{
  return _name;
}

Particle Sampling::random_sample(nav_msgs::OccupancyGrid map)
{
  geometry_msgs::PoseWithCovarianceStamped pose_with_cov;

  double x = map.info.origin.position.x + _distribution(_gen) * (map.info.width * map.info.resolution);

  double y = map.info.origin.position.y + _distribution(_gen) * (map.info.height * map.info.resolution);
  double theta = -M_PI + _distribution(_gen) * (2*M_PI);

  pose_with_cov.pose.pose.position.x = x;
  pose_with_cov.pose.pose.position.y = y;
  pose_with_cov.pose.pose.orientation.z = std::sin(theta / 2);
  pose_with_cov.pose.pose.orientation.w = std::cos(theta / 2);
  Particle p(0.0, pose_with_cov);
  OccupancyMapPtr a_map(new OccupancyMap);
  a_map->create_map(map.info.width, map.info.height, map.info.resolution);
  p.set_map(a_map);
  return p;
}

std::vector<Particle> Sampling::renormalize(std::vector<Particle> particles)
{
  double total_weight = 0.0;
  int size = particles.size();
  for (auto it = particles.begin(); it < particles.end(); ++it)
  {
    total_weight += it->get_weight();
  }

  for (auto it = particles.begin(); it < particles.end(); ++it)
  {
    if (total_weight != 0)
    {
      it->set_weight(it->get_weight() / total_weight);
    }else{
      it->set_weight(1.0 / ((double) size));
    }
  }
  return particles;
}
