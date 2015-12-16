/**
 * @file particle_filter.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include "localization/util/particle.h"
#include "localization/filtering_strategies/pose_filter.h"


#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

/**
 * @brief Implementation of the particle filter algorithm.
 *
 */
class ParticleFilter: public PoseFilter
{
public:
  ParticleFilter(); ///< Default Constructor

  /**
   * Class constructor.
   * @param number_of_particles number of particles.
   * @param random_particles number of random particles to init in every iteration.
   */
  ParticleFilter(int number_of_particles, int random_particles, double weight_threshold);

  /**
   * Execute motion update step.
   * @param msg odometry msg pointer.
   */
  void motion_update(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * Execute sensor correcting step.
   * @param msg laser msg pointer.
   */
  void sensor_update(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * Execute resampling algorithm
   */
  void resample();

  /**
   * Retrieve the estimated poses
   * @return vector of poses with covariances.
   */
  std::vector<geometry_msgs::PoseWithCovarianceStamped> get_poses();

  /**
   * Get the best pose
   * @return pose with covariance
   */
  geometry_msgs::PoseWithCovarianceStamped get_best_pose();

  /**
   * Set map to the algorithm
   * @param msg occupancy grid msg pointer.
   */
  void set_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
   * Set the number of particles to the algorithm.
   * @param number_of_particles
   */
  void set_number_of_particles(int number_of_particles);

  /**
   * Set the number of random particles to be reset in each iteration.
   * @param random_particles
   */
  void set_random_particles(int random_particles);

  /**
   * Reset the particles.
   */
  void reset_particles();

  /**
   * Get the particles.
   * @return vector of particles.
   */
  std::vector<Particle> get_particles();

  /**
   * Check if filter is confident
   * @return
   */
  bool is_good();

private:
  double _weight_threshold; ///< Minimum weight of particle to be considered good.
  std::vector<Particle> _particles; ///< vector of particles.
  int _number_of_particles; ///< number of particles
  int _random_particles; ///< number of vector particles
  nav_msgs::Odometry::ConstPtr _old_odometry_msg; ///< last odometry msg.
};


#endif /* PARTICLE_FILTER_H_ */
