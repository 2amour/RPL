/**
 * @file stochastic_universal_sampling.cpp
 *
 *  @date Dec 4, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */

#include "localization/sampling_strategies/stochastic_universal_sampling.h"

static const std::string NAME = "Stochastic Universal Sampling";

StochasticUniversalSampling::StochasticUniversalSampling()
{
  _name = NAME;
  _gen.seed(rd());
}

std::vector<Particle> StochasticUniversalSampling::resample(std::vector<Particle> particles, int number_of_particles,
                                                            nav_msgs::OccupancyGrid map)
{
  std::vector<Particle> out;
  out.assign(number_of_particles, Particle());

  std::vector<double> cumulated_probabilities(particles.size(), 0.0);
  for (int i = 1; i < particles.size(); ++i)
  {
    cumulated_probabilities[i] = cumulated_probabilities[i - 1] + particles[i - 1].get_weight();
  }

  if (particles.size() > 0)
  {
    double probability = _distribution(_gen) *  1.0 / (double)particles.size();
    for (int i = 0; i < particles.size(); ++i)
    {
      auto lower = std::lower_bound(cumulated_probabilities.begin(), cumulated_probabilities.end(), probability);
      int idx = (int)(lower - cumulated_probabilities.begin()-1);
      probability += 1.0 / (double)particles.size();
      out[i] = particles[idx];
    }
  }

  for (int i = particles.size(); i < number_of_particles; ++i)
  {
    out[i] = random_sample(map);
    out.back().set_weight(1.0 / ((double)number_of_particles));
  }

  return renormalize(out);
}
