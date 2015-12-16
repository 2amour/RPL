/*
 * motion_updater.cpp
 *
 *	Motion updater for localization algorithm.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#include <algorithms/motion_updater/motion_updater.h>

// Motion updater destructor.
MotionUpdater::~MotionUpdater ()
{
}

// Odometry motion model constructor.
OdometryMotionModel::OdometryMotionModel (const OdometryMotionModelParams& params_input, const boost::shared_ptr<ProbabilityDistribution>& params_sampler):
		params(params_input), sampler (params_sampler)
{
}

// Odometry motion model destructor.
OdometryMotionModel::~OdometryMotionModel ()
{
}

// Odometry motion model update.
Pose<float>
OdometryMotionModel::motionUpdate (const Control<float>& control, const Pose<float>& initial_pose)
{
	float rot1, trans, rot2;
	float rot1_, trans_, rot2_;

	float a1 = params.a1;
	float a2 = params.a2;
	float a3 = params.a3;
	float a4 = params.a4;

	// Input pose at time t.
	float x1 = control.array[1].position[0];
	float y1 = control.array[1].position[1];
	float theta1 = control.array[1].orientation[0];

	// Input pose at time t-1.
	float x0 = control.array[0].position[0];
	float y0 = control.array[0].position[1];
	float theta0 = control.array[0].orientation[0];

	// Initial pose.
	float x = initial_pose.position[0];
	float y = initial_pose.position[1];
	float theta = initial_pose.orientation[0];

	double diffx = x1-x0;
	double diffy = y1-y0;

	// Recover relative motion parameters from robot control.
	rot1 = atan2(diffy, diffx) - theta0;
	trans = sqrt( pow(diffx,2) + pow(diffy,2));
	rot2 = theta1 - theta0 - rot1;

	// Perturb the motion parameters by noise in robot motion.
	float rot1_sq = pow(rot1,2);
	float rot2_sq = pow(rot2,2);
	float trans_sq = pow(trans,2);

	rot1_ = rot1 - sampler->sample(a1*rot1_sq + a2*trans_sq);
	trans_ = trans - sampler->sample(a3*trans_sq + a4*rot1_sq + a4*rot2_sq);
	rot2_ = rot2 - sampler->sample(a1*rot2_sq + a2*trans_sq);

	// Update the output pose using the sample motion parameters.
	float x_, y_, theta_;
	x_ = x + trans_*cosf(theta + rot1_);
	y_ = y + trans_*sinf(theta + rot1_);
	theta_ = theta + rot1_ + rot2_;

	float position [2] = {x_, y_};
	float orientation [1] = {theta_};
	Pose<float> result (position, orientation);

	return result;
}



