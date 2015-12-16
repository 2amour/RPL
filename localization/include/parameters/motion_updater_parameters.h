/*
 * motion_updater_parameters.h
 *
 *	Parameters for the motion updater algorithm.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_PARAMETERS_MOTION_UPDATER_PARAMETERS_H_
#define LOCALIZATION_INCLUDE_PARAMETERS_MOTION_UPDATER_PARAMETERS_H_

// Odometry motion model algorithm parameters.
struct OdometryMotionModelParams
{
	float a1, a2, a3, a4;
};

#endif /* LOCALIZATION_INCLUDE_PARAMETERS_MOTION_UPDATER_PARAMETERS_H_ */
