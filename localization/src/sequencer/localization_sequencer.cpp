/*
 * localization_sequencer.cpp
 *
 *	Sequencer for the localization process.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#include "sequencer/localization_sequencer.h"

// Localization sequencer constructor, initializes algorithm, current particle, sensor input and map_ready flag.
LocalizationSequencer::LocalizationSequencer (const LocalizerParameters& params_bag):
	algorithm (params_bag.algorithm), x_t(), z_t()
{
	// Initialize flag.
	map_ready = false;

	// Sets a visualizer for the algorithm output.
	algorithm->setVisualizer(params_bag.visualizer);
}

// Localization sequencer destructor.
LocalizationSequencer::~LocalizationSequencer ()
{
}

// Localization sequencer start.
void
LocalizationSequencer::start ()
{
	// Start localization.
	algorithm->localize(x_t, z_t);
}

// Localization sequencer send map to algorithm.
void
LocalizationSequencer::sendMap (const MsgMap::ConstPtr& map_input)
{
	// Set map to ready.
	map_ready = true;

	// Set map for algorithm.
	algorithm->setMap(map_input);
}

// Localization sequencer update.
void
LocalizationSequencer::update (const MsgOdometry::ConstPtr& motion, const MsgLaserScan::ConstPtr& sensor)
{
	// Position and orientation of the pose message.
	float position [3], orientation [3];

	// Extract angles from odometry message.
	tf::Quaternion q;
	q.setValue(motion->pose.pose.orientation.x, motion->pose.pose.orientation.y, motion->pose.pose.orientation.z, motion->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double yaw, pitch, roll;
	m.getEulerYPR(yaw, pitch, roll);

	// Create Pose from odometry message.
	position [0] = motion->pose.pose.position.x;
	position [1] = motion->pose.pose.position.y;
	position [2] = motion->pose.pose.position.z;
	orientation[0] = (float)yaw;
	orientation[1] = (float)pitch;
	orientation[2] = (float)roll;

	// Create Pose.
	boost::shared_ptr<Pose<float> > ptr_x_t (new Pose<float>(position, orientation));

	// Store input.
	x_t = ptr_x_t; ///< Store pose at time t.
	z_t = sensor; ///< Store sensor msg at time t.

	// If map is set then start localization.
	if (map_ready)
		{
			start ();
		}
}


