/**
 * @file roulette_wheel_sampling.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sampling_strategies/roulette_wheel_sampling.h"

static const std::string NAME = "Roulette Wheel Sampling";

RouletteWheelSampling::RouletteWheelSampling()
{
  _name = NAME;
  _gen.seed(rd());
}

std::vector<Particle> RouletteWheelSampling::resample(std::vector<Particle> particles, int number_of_particles,
                                                      nav_msgs::OccupancyGrid map)
{
  std::vector<Particle> out;
  out.assign(number_of_particles, Particle());

  std::vector<double> cumulated_probabilities(particles.size(), 0.0);
  for (int i = 1; i < particles.size(); ++i)
  {
    cumulated_probabilities[i] = cumulated_probabilities[i - 1] + particles[i - 1].get_weight();
  }

  for (int i = 0; i < particles.size(); ++i)
  {
   auto lower = std::lower_bound(cumulated_probabilities.begin(), cumulated_probabilities.end(),
                                    _distribution(_gen) * cumulated_probabilities.back());
   int idx = (int)(lower - cumulated_probabilities.begin());
   out[i] = particles[idx];
  }

  for (int i = particles.size(); i < number_of_particles; ++i)
  {
    out[i] = random_sample(map);
    out.back().set_weight(1.0 / ((double)number_of_particles));
  }

  return renormalize(out);
}
