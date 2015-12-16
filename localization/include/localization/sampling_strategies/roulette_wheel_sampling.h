/**
 * @file roulette_wheel_sampling.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include "sampling.h"
#include <algorithm>

#ifndef ROULETTE_WHEEL_SAMPLING_H_
#define ROULETTE_WHEEL_SAMPLING_H_

/**
 * @brief Roulette wheel implementation of sampling strategy pattern for particles.
 *
 */
class RouletteWheelSampling: public Sampling
{
public:
  RouletteWheelSampling(); ///< Default Constructor

  /**
   * Resampling according to strategy.
   * @param particles initial vector of particles.
   * @param number_of_particles number of output particles.
   * @param map map where particles should be sampled.
   * @return output resampled particles.
   */
  std::vector<Particle> resample(std::vector<Particle> particles, int number_of_particles, nav_msgs::OccupancyGrid map);
};

#endif /* ROULETTE_WHEEL_SAMPLING_H_ */
