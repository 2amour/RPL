/*
 * resampler.cpp
 *
 *	Resampler algorithm.
 *  Created on: Dec 5, 2015
 *      Author: toni
 */

#include <algorithms/resampler/resampler.h>

// Virtual destructor.
Resampler::~Resampler()
{
}

// Constructor for roulette wheel sampler.
RWS::RWS(const RWSParams& params_input)
{
}

// Destructor for roulette wheel sampler.
RWS::~RWS()
{
}

// Resample member of roulette wheel sampler.
void
RWS::resample(boost::shared_ptr<std::vector<Particle> >& sample, float total_cumulated_weight)
{
	float randomised_weight = 0;
	float actual_cumulated_weight = 0;
	float random_number = 0;

	// New samples.
	std::vector<Particle> new_samples (sample->size());

	// Create new sample.
	if (total_cumulated_weight !=0 )
		{
			for (int i = 0; i < sample->size(); ++i)
				{
					// Compute a randomised weight.
					random_number = (float)std::rand()/RAND_MAX;
					randomised_weight = random_number*total_cumulated_weight;

					// New particle is where the one which has an accumulated weight higher than the randomised weight is.
					for (int j = 0; j < sample->size(); ++j)
						{
							actual_cumulated_weight += sample->at(j).weight;
							if (actual_cumulated_weight >= randomised_weight)
								{
									new_samples.at(i) = sample->at(j);
									break;
								}
							else
								{
									continue;
								}
						}
					actual_cumulated_weight = 0;
				}
			*sample = new_samples;
		}
}
