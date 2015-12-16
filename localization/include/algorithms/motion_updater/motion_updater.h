/*
 * motion_updater.h
 *
 *	Motion updater for localization algorithm.
 *
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_ALGORITHMS_MOTION_UPDATER_MOTION_UPDATER_H_
#define LOCALIZATION_INCLUDE_ALGORITHMS_MOTION_UPDATER_MOTION_UPDATER_H_

#include <math.h>
#include <parameters/motion_updater_parameters.h>
#include <types/pose.h>
#include <boost/shared_ptr.hpp>
#include "probability_distributions/probability_distributions.h"

// Virtual class for motion updaters.
class MotionUpdater
{
public:

	// Virtual destructor for motion updater.
	virtual ~MotionUpdater ();

	// Virtual function for updating motion.
	virtual Pose<float> motionUpdate (const Control<float>& control, const Pose<float>& initial_pose) = 0;
};

// Sample odometry motion model.
class OdometryMotionModel: public MotionUpdater
{
public:

	// Odometry motion model constructor.
	OdometryMotionModel (const OdometryMotionModelParams& params_input, const boost::shared_ptr<ProbabilityDistribution>& params_sampler);

	// Virtual destructor for odometry motion model.
	virtual ~OdometryMotionModel ();

	// Updating update.
	Pose<float> motionUpdate (const Control<float>& control, const Pose<float>& initial_pose);

private:

	// Sampler with probability distribution.
	boost::shared_ptr<ProbabilityDistribution> sampler;

	// Parameters
	const OdometryMotionModelParams params;
};

#endif /* LOCALIZATION_INCLUDE_ALGORITHMS_MOTION_UPDATER_MOTION_UPDATER_H_ */
