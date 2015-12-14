/*
 * localization_sequencer.h
 *
 *	Sequencer for the localization process.
 *
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_SEQUENCER_LOCALIZATION_SEQUENCER_H_
#define LOCALIZATION_INCLUDE_SEQUENCER_LOCALIZATION_SEQUENCER_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "parameters/localizer_parameters.h"
#include "algorithms/localizer/localization_algorithm.h"
#include "types/pose.h"

typedef nav_msgs::OccupancyGrid MsgMap; ///< Message for map.
typedef nav_msgs::Odometry MsgOdometry; ///< Message for odometry.
typedef sensor_msgs::LaserScan MsgLaserScan; ///< Message for laser scan.

// Sequencer for localization.
class LocalizationSequencer
{
public:

	// Constructor.
	LocalizationSequencer (const LocalizerParameters& params_bag);

	// Destructor.
	virtual ~LocalizationSequencer ();

	// Send map to algorithm.
	void sendMap (const MsgMap::ConstPtr& map_input);

	// Update inputs.
	void update (const MsgOdometry::ConstPtr& motion, const MsgLaserScan::ConstPtr& sensor);

private:

	// Start localization.
	void start ();

	// Algorithm used.
	boost::shared_ptr<LocalizationAlgorithm> algorithm;

	// Odometry given pose at time t.
	boost::shared_ptr<Pose<float> > x_t;

	// Sensor input at time t;
	MsgLaserScan::ConstPtr z_t;

	// Flag to check if the map is loaded.
	bool map_ready;

};

#endif /* LOCALIZATION_INCLUDE_SEQUENCER_LOCALIZATION_SEQUENCER_H_ */
