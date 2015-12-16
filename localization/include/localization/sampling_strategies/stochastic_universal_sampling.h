/**
 * @file stochastic_universal_sampling.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */

#include "sampling.h"

#ifndef STOCHASTIC_UNIVERSAL_SAMPLING_H_
#define STOCHASTIC_UNIVERSAL_SAMPLING_H_

/**
 * @brief Stochastic universal implementation of sampling strategy pattern for particles.
 *
 */
class StochasticUniversalSampling: public Sampling
{
public:
  StochasticUniversalSampling(); ///< Default Constructor
   /**
   * Resampling according to strategy.
   * @param particles initial vector of particles.
   * @param number_of_particles number of output particles.
   * @param map map where particles should be sampled.
   * @return output resampled particles.
    */
  std::vector<Particle> resample(std::vector<Particle> particles, int number_of_particles, nav_msgs::OccupancyGrid map);
};

#endif /* STOCHASTIC_UNIVERSAL_SAMPLING_H_ */
