/**
 * @file sampling.h
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bug No known bug.
 */
#include <localization/util/particle.h>
#include <random>


#ifndef SAMPLING_H_
#define SAMPLING_H_

/**
 * @brief Abstract class for implementing the sampling strategy pattern
 *
 */
class Sampling
{
public:
  virtual ~Sampling() {}; ///< Virtual Deconstructor

  /**
   * Resampling according to strategy.
   * @param particles initial vector of particles.
   * @param number_of_particles number of output particles.
   * @param map map where particles should be sampled.
   * @return output resampled particles.
   */
  virtual std::vector<Particle> resample(std::vector<Particle> particles, int number_of_particles, nav_msgs::OccupancyGrid map) = 0;

  /**
   * Get sampling algorihtm name.
   * @return output set of particles.
   */
  std::string get_name();

  /**
   * Get random particle from map
   * @param map map where particles should be sampled.
   * @return
   */
  Particle random_sample(nav_msgs::OccupancyGrid map);

  /**
   * Renormalize a vector of particles so sum(weights)=1
   * @param vector vector of particles.
   * @return renormalized vector of particles .
   */
  std::vector<Particle> renormalize(std::vector<Particle> vector);
protected:
  std::string _name; ///< sampling algorithm name.
  std::random_device rd; ///< random device.
  std::mt19937 _gen; ///< random number generator.
  std::uniform_real_distribution<double> _distribution; ///<uniform distribution generator between 0 and 1.
};

/**
 * @brief Shared pointer to sampling class
 */
typedef boost::shared_ptr<Sampling> SamplingPtr;


#endif /* SAMPLING_H_ */
