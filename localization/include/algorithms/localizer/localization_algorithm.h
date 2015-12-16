/*
 * localization_algorithm.h
 *
 *	Localizer algorithm for localization.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_ALGORITHMS_LOCALIZER_LOCALIZATION_ALGORITHM_H_
#define LOCALIZATION_INCLUDE_ALGORITHMS_LOCALIZER_LOCALIZATION_ALGORITHM_H_

#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "parameters/parameters_bag.h"
#include "algorithms/motion_updater/motion_updater.h"
#include "algorithms/sensor_updater/sensor_updater.h"
#include "algorithms/resampler/resampler.h"
#include "types/pose.h"
#include "visualizer/visualizer.h"

typedef nav_msgs::OccupancyGrid MsgMap; ///< Message for map.
typedef sensor_msgs::LaserScan MsgLaserScan; ///< Message for laser scan.

// Deferred class for localization algorithms.
class LocalizationAlgorithm {
public:

	// Virtual destructor for localization algorithm.
	virtual ~LocalizationAlgorithm ();

	// Set map of loclaization algorithm.
	virtual void setMap(const MsgMap::ConstPtr& map_input) = 0;

	// Set visualizer for publishing output.
	virtual void setVisualizer(const boost::shared_ptr<Visualizer>& vis) = 0;

	// Start localization algorithm, returns a boolean saying if we are localized.
	virtual bool localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t, const boost::shared_ptr<Pose<float> >& localization) = 0;

	// Reset algorithm.
	virtual void reset () = 0;

};

// Particle Filter localization algortithm.
class ParticleFilter: public LocalizationAlgorithm {
public:

	// Particle filter algorithm constructor.
	ParticleFilter(const ParticleFilterParameters& params);

	// Particle filter algorithm destructor
	virtual ~ParticleFilter();

	// Set map for the algorithm.
	void setMap(const MsgMap::ConstPtr& map_input);

	// Set visualizer for publishing output.
	void setVisualizer(const boost::shared_ptr<Visualizer>&);

	// Start localization algorithm.
	bool localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t, const boost::shared_ptr<Pose<float> >& localization);

	// Reset algorithm.
	void reset ();

private:

	// Set of particles.
	boost::shared_ptr<std::vector<Particle> > particles_ptr;

	// Control input at time t.
	boost::shared_ptr<Control<float> > u_t ;

	// Odometry input at time t.
	boost::shared_ptr<Pose<float> > x_t1 ;

	// Odometry input at time t - 1.
	boost::shared_ptr<Pose<float> > x_t0 ;

	// Motion updater.
	boost::shared_ptr<MotionUpdater> p_motion_updater;

	// Sensor updater.
	boost::shared_ptr<SensorUpdater> p_sensor_updater;

	// Resampler.
	boost::shared_ptr<Resampler> p_resampler;

	// Particles publisher.
	boost::shared_ptr<Visualizer> p_visualizer;

	// Are we localized?
	bool getLocalization (const boost::shared_ptr<Pose<float> >& localization);

	// Get particles from map.
	void getParticlesFromMap ();

	// Map.
	MsgMap::ConstPtr map;

	// Make a local map.
	void makeLocalMap (const MsgLaserScan::ConstPtr& sensor, const boost::shared_ptr<Pose<float> >& curr_pose);

	// Local map.
	std::vector<Particle> local_map;

	// Number of jumps between map cells for first sampling of the particles set.
	int cells_jumped;

	// Angle step between orientations of the initial particles in the map.
	float delta_theta;

	// Cumulated weight of particles;
	float cw;

	// Threshold for deciding whether we are localized or not.
	float angle_threshold, position_threshold;

	// Map properties.
	int width, height, size;
	float resolution;
};

#endif /* LOCALIZATION_INCLUDE_ALGORITHMS_LOCALIZER_LOCALIZATION_ALGORITHM_H_ */
