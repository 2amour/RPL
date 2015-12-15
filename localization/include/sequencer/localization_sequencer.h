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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "parameters/localizer_parameters.h"
#include "algorithms/localizer/localization_algorithm.h"
#include "types/pose.h"
#include "visualizer/visualizer.h"
#include "types/pose.h"


typedef nav_msgs::OccupancyGrid MsgMap; ///< Message for map.
typedef nav_msgs::Odometry MsgOdometry; ///< Message for odometry.
typedef sensor_msgs::LaserScan MsgLaserScan; ///< Message for laser scan.
typedef boost::shared_ptr<std_msgs::Bool> onOff; /// < Flag for on/off the localization.

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

	// Stores on off flag.
	void onOffCallback (const onOff& on_off);

	// Publishes a transform between base link and map so that base link is over the believed location.
	void publishOdometry (const MsgOdometry::ConstPtr& motion);

private:

	// Start localization.
	void start ();

	// Pose of the localized robot.
	boost::shared_ptr<Pose<float> > localization;

	// Particles publisher.
	boost::shared_ptr<LocalizationPublisher> localization_pub;

	// Flag publisher for telling if we are localized.
	ros::Publisher flag_pub;

	// Algorithm used.
	boost::shared_ptr<LocalizationAlgorithm> algorithm;

	// Control input at time t.
	boost::shared_ptr<Control<float> > u_t ;

	// Odometry given pose at time t.
	boost::shared_ptr<Pose<float> > x_t;

	// Odometry input at time t.
	MsgOdometry::ConstPtr m_t;

	// Sensor input at time t.
	MsgLaserScan::ConstPtr z_t;

	// Flag message for if we are localized.
	std_msgs::Bool isLocalized;

	// Flag to check if the map is loaded.
	bool map_ready;

	// Is localized for the first time.
	bool isFirstTimeLocalized;

	// Flag for enabling or disabling the localization algorithm
	bool is_on;

	// Updates odometry of the initial localization pose using the control input to the robot.
	void updatePose (const Control<float>& control, Pose<float>& initial_pose);

};

#endif /* LOCALIZATION_INCLUDE_SEQUENCER_LOCALIZATION_SEQUENCER_H_ */
