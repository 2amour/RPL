/*
 * particle_filter_localizer.cpp
 *
 * 	Main application for particle filtering localization algorithm for robot localization.
 * 	Input:
 * 		- a map.
 * 		- motion information.
 * 		- parameters.
 *	Output:
 *		- Visualized set of poses in 2D.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

#include "parameters/parameters_bag.h"
#include "io/parser_sequencer.h"
#include "sequencer/localization_sequencer.h"
#include "visualizer/visualizer.h"

typedef nav_msgs::OccupancyGrid MsgMap; ///< Message for map.
typedef nav_msgs::Odometry MsgOdometry; ///< Message for odometry.
typedef sensor_msgs::LaserScan MsgLaserScan; ///< Message for laser scan.
typedef message_filters::sync_policies::ApproximateTime<MsgOdometry, MsgLaserScan> SyncPolicy; ///< Synchronization type set to approximate, timestamps of messages are similar.

int main(int argc, char** argv)
{
  // Initialize ROS.
  ros::init (argc, argv, "localizer");
  ros::NodeHandle nh;

  // Get parameters.
	ParametersBag params_bag;
	ParserSequencer parser;

	if (parser.parseAll (params_bag))
	{
		std::cerr << "Error call in parser sequencer, exiting application." << std::endl;
		exit (-1);
	}

  // Creates a publisher for poses and sends it to the visualizer.
  ros::Publisher visualizer_pub = nh.advertise<geometry_msgs::PoseArray>(params_bag.topics.publisher, 100);
  params_bag.localizer.visualizer->setPublisher(visualizer_pub);

  // Initialize localization sequencer.
	LocalizationSequencer localizer (params_bag.localizer);

  // Get map.
  message_filters::Subscriber<MsgMap> map_sub(nh, params_bag.topics.map, 1);
  map_sub.registerCallback(boost::bind(&LocalizationSequencer::sendMap, &localizer, _1));

	// Subscribe to odometry.
	message_filters::Subscriber<MsgOdometry> motion_sub(nh, params_bag.topics.odometry, 1);

	// Subscribe to sensor.
	message_filters::Subscriber<MsgLaserScan> sensor_sub(nh, params_bag.topics.sensor, 1);

	// Set synchronisation of messages.
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), motion_sub, sensor_sub);

	// Register callback.
	sync.registerCallback(boost::bind(&LocalizationSequencer::update, &localizer, _1, _2));

	// Spin.
	ros::spin();

  return 0;
}
