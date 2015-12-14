/*
 * probability_distributions.h
 *
 *	Probability distributions.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_PROBABILITY_DISTRIBUTIONS_PROBABILITY_DISTRIBUTIONS_H_
#define LOCALIZATION_INCLUDE_PROBABILITY_DISTRIBUTIONS_PROBABILITY_DISTRIBUTIONS_H_

#include <iostream>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <cmath>

// Get random.
float
getRandom (float num);

// Virtual probability distribution.
class ProbabilityDistribution
{
public:

	// Virtual probability distribution.
	virtual ~ProbabilityDistribution();

	// Draw sample using a probabilistic distribution.
	virtual float sample (float b_sq) = 0;
};

// Approximate normal distribution.
class ApproxNormalDistribution: public ProbabilityDistribution
{
public:

	// Approximate normal distribution constructor.
	ApproxNormalDistribution();

	// Virtual approximate normal distribution destructor.
	virtual ~ApproxNormalDistribution();

	// Draw sample using an approximate normal distribution as probabilistic distribution.
	float sample (float b_sq);
};

// Triangular distribution
class TriangularDistribution: public ProbabilityDistribution
{
public:

	// Triangular distribution constructor.
	TriangularDistribution();

	// Virtual triangular distribution.
	virtual ~TriangularDistribution();

	// Draw sample using a triangular distribution as probabilistic distribution.
	float sample (float b_sq);
};

#endif /* LOCALIZATION_INCLUDE_PROBABILITY_DISTRIBUTIONS_PROBABILITY_DISTRIBUTIONS_H_ */
