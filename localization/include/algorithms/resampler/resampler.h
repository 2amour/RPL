/*
 * resampler.h
 *
 *	Resampling algorithms.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_ALGORITHMS_RESAMPLER_RESAMPLER_H_
#define LOCALIZATION_INCLUDE_ALGORITHMS_RESAMPLER_RESAMPLER_H_

#include "parameters/resampler_parameters.h"
#include "types/pose.h"
#include <boost/shared_ptr.hpp>
#include <vector>

// Resampler generic class.
class Resampler
{
public:

	// Virtual destructor.
	virtual ~Resampler ();

	// Pure virtual resampler.
	virtual void resample (boost::shared_ptr<std::vector<Particle> >& sample, float cumulated_weight) = 0;

};

// Roulette wheel sampler.
class RWS: public Resampler
{
public:

	// Roulette wheel sampler constructor.
	RWS (const RWSParams& params_input);

	// Roulette wheel sampler destructor.
	virtual ~RWS ();

	// Resample set of particles.
	void resample (boost::shared_ptr<std::vector<Particle> >& sample, float cumulated_weight);

};


#endif /* LOCALIZATION_INCLUDE_ALGORITHMS_RESAMPLER_RESAMPLER_H_ */
