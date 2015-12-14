/*
 * probability_distributions.cpp
 *
 *	Probability distributions.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#include "probability_distributions/probability_distributions.h"

// Return a random number between -num and num.
float
getRandom (float num)
{
	float a = (float)std::rand()/RAND_MAX;
	float rand = a*(2*num)-num;
	return rand;
}

// Probability distribution destructor.
ProbabilityDistribution::~ProbabilityDistribution()
{
}

// Approximated normale distribiution constructor.
ApproxNormalDistribution::ApproxNormalDistribution()
{
}

// Approximated normale distribiution destructor.
ApproxNormalDistribution::~ApproxNormalDistribution()
{
}

// Draw sample with variance of square root of b_sq.
float
ApproxNormalDistribution::sample (float b_sq)
{
	float sum = 0;
	for (int i = 1; i<= 12; ++i)
		{
			sum += getRandom (sqrtf(b_sq));
		}
	sum = sum / 2;
	return sum;
}

// Triangular distribution constructor.
TriangularDistribution::TriangularDistribution()
{
}

// Triangular distribution destructor.
TriangularDistribution::~TriangularDistribution()
{
}

// Draw a sample with variance of square root of b_sq
float
TriangularDistribution::sample (float b_sq)
{
	float result = 0;
	result = (float)sqrtf(6)/2*(getRandom(sqrtf(b_sq))+getRandom(sqrtf(b_sq)));
	return result;
}

