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

	// Start loclaization algorithm.
	virtual void localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t) = 0;
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
	void localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t);

private:

	// Set of particles.
	boost::shared_ptr<std::vector<Particle> > particles_ptr;

	// Control input at time t.
	boost::shared_ptr<Control<float> > u_t ;

	// Motion updater.
	boost::shared_ptr<MotionUpdater> p_motion_updater;

	// Sensor updater.
	boost::shared_ptr<SensorUpdater> p_sensor_updater;

	// Resampler.
	boost::shared_ptr<Resampler> p_resampler;

	// Particles publisher.
	boost::shared_ptr<Visualizer> p_visualizer;

	// Map.
	MsgMap::ConstPtr map;

	// Number of jumps between map cells for first sampling of the particles set.
	int cells_jumped;

	// Map properties.
	int width, height, size;
	float resolution;

	// Get particles from map.
	void getParticlesFromMap ();

};

#endif /* LOCALIZATION_INCLUDE_ALGORITHMS_LOCALIZER_LOCALIZATION_ALGORITHM_H_ */
