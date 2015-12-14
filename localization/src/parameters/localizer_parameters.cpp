/*
 * localizer_parameters.cpp
 *
 *	Localizer parameters.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#include "parameters/localizer_parameters.h"

// Get set of parameters.
std::vector<std::string>
ParticleFilterParameters::getParams()
{
	std::vector<std::string> params;
	return params;
}

// Particle filter parameters destructor.
ParticleFilterParameters::~ParticleFilterParameters ()
{
}

// Algorithm parameters destructor.
AlgorithmParameters::~AlgorithmParameters ()
{
}
