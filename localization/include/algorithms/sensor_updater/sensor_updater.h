/*
 * sensor_updater.h
 *
 *	Sensor updater for localization algorithm.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_ALGORITHMS_SENSOR_UPDATER_SENSOR_UPDATER_H_
#define LOCALIZATION_INCLUDE_ALGORITHMS_SENSOR_UPDATER_SENSOR_UPDATER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include "parameters/sensor_updater_parameters.h"
#include "types/pose.h"

typedef nav_msgs::OccupancyGrid MsgMap; ///< Message for map.
typedef sensor_msgs::LaserScan MsgLaserScan; ///< Message for laser scan.

// Virtual class for sensor updaters.
class SensorUpdater
{
protected:

	// Cumulated weight of the particles.
	float cumulated_weight;

public:

	// Virtual destructor.
	virtual ~SensorUpdater ();

	// Sensor upsate.
	virtual float sensorUpdate (const MsgLaserScan::ConstPtr& z_t, const Pose<float>& current_pose) = 0;

	// Set input map.
	virtual void setMap (const MsgMap::ConstPtr& map_input) = 0;

	// Get cumulated_weight.
	float getCumulatedWeight ();

	// Get cumulated_weight.
	void setCumulatedWeight (float);

};

// Sample odometry motion model.
class MapMatching: public SensorUpdater
{
public:

	// Constructor.
	MapMatching (const MapMatchingParams& params_input);

	// Destructor.
	virtual ~MapMatching ();

	// Sensor update.
	float sensorUpdate (const MsgLaserScan::ConstPtr& z_t, const Pose<float>& current_pose);

	// Set input map.
	void setMap (const MsgMap::ConstPtr& map_input);

private:

	// Past pose.
	Pose<float> past_pose;

	// Global map.
	MsgMap::ConstPtr global_map;

	// Local map.
	MsgMap::ConstPtr local_map;

	// Global map properties.
	int global_width;
	int global_height;
	float global_resolution;

};

#endif /* LOCALIZATION_INCLUDE_ALGORITHMS_SENSOR_UPDATER_SENSOR_UPDATER_H_ */
