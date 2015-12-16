/**
 * @file particle.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "occupancy_map.h"
/**
 * @brief Class that implements a particle for a SLAM algorithm.
 *
 */
class Particle
{
public:
  Particle(); ///< Default Constructor

  /**
   * Class constructor.
   * @param weight particle belief.
   * @param pose particle pose.
   */
  Particle(double weight, geometry_msgs::PoseWithCovarianceStamped pose);
  double get_weight(); ///< get particle belief
  geometry_msgs::PoseWithCovarianceStamped get_pose(); ///< get particle pose
  OccupancyMapPtr get_map(); ///< get particle local map.

  /**
   * Set particle belief
   * @param weight
   */
  void set_weight(double weight);

  /**
   * Set particle pose
   * @param pose
   */
  void set_pose(geometry_msgs::PoseWithCovarianceStamped pose);

  /**
   * Set particle map
   * @param map
   */
  void set_map(OccupancyMapPtr map);
private:
  double _weight; ///< particle weight.
  geometry_msgs::PoseWithCovarianceStamped _pose; ///< particle map.
  OccupancyMapPtr _map; ///< local map copy.
};

/**
 * @brief Shared pointer to particles class
 */
typedef boost::shared_ptr<Particle> ParticlePtr;

#endif /* PARTICLE_H_ */
