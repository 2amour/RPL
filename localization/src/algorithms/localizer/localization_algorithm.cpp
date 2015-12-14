/*
 * localization_algorithm.cpp
 *
 * 	Interface for localization algorithms.
 *
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#include "algorithms/localizer/localization_algorithm.h"

// Localization algorithm destructor.
LocalizationAlgorithm::~LocalizationAlgorithm ()
{
}

// Particle filter algorithm constructor.
ParticleFilter::ParticleFilter (const ParticleFilterParameters& params):
		p_motion_updater(params.motion_updater), p_sensor_updater(params.sensor_updater), p_resampler (params.resampler), u_t(new Control<float>), p_visualizer()
{
		height = 0;
		width = 0;
		size = 0;
		resolution = 0;
		cells_jumped = params.cells_jumped;
}

// Particle filter destructor.
ParticleFilter::~ParticleFilter ()
{
}

// Set input map for the localization algorithm.
void
ParticleFilter::setMap (const MsgMap::ConstPtr& map_input)
{
	map = map_input;
	width = map->info.width;
	height = map->info.height;
	resolution = map->info.resolution;
	size = map->data.size();
	p_sensor_updater->setMap(map);
	getParticlesFromMap ();
}

// Make an instance of particles from map.
void
ParticleFilter::getParticlesFromMap ()
{
	// Create particles set.
	Particle particle;

	// Shared pointer of set of particles.
	boost::shared_ptr<std::vector<Particle> > particles (new std::vector<Particle>);

	// Create intial particles in the map.
	for (int i = 0; i < width; i += cells_jumped)
		{
			for (int j = 0; j < height; j += cells_jumped)
				{
					if (map->data[i+j*width]==0)
						{
							for (float theta = 0; theta < 2*M_PI; theta= theta +0.15)
								{
									particle.pose.position[0] = resolution*(i);
									particle.pose.position[1] = resolution*(j);
									particle.pose.orientation[0] = theta - M_PI;
									particles->push_back(particle);
								}
						}
				}
	  }

	// Store initial particles.
	particles_ptr = particles;
}

// Start algorithm.
void
ParticleFilter::localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t)
{
	// Set control input.
	u_t->array[0] = u_t->array[1]; ///< Store pose at time t-1.
	u_t->array[1] = *x_t; ///< Store pose at time t.

	// Map properties.
	int width = map->info.width;
	int height = map->info.height;
	float resolution = map->info.resolution;

	// Checks whether the pose is the same as before.
	bool same_pose = u_t->array[1].position == u_t->array[0].position && u_t->array[1].orientation == u_t->array[0].orientation;

	// Start algorithm only if the pose is different than previous.
	if (!same_pose)
		{
			for (int i = 0; i < particles_ptr->size(); ++i)
				{
					// Motion update.
					particles_ptr->at(i).pose = p_motion_updater->motionUpdate(*u_t, particles_ptr->at(i).pose);

					// Sensor update.
					particles_ptr->at(i).weight = p_sensor_updater->sensorUpdate(z_t, particles_ptr->at(i).pose);
				}

			// Resampling.
			p_resampler->resample(particles_ptr, p_sensor_updater->getCumulatedWeight());

			// Reinitialization of cumulated_weight.
			p_sensor_updater->setCumulatedWeight(0);
		}

	// Visualize particles.
	p_visualizer->publishParticles(*particles_ptr);
}

// Sets the visualizer for visualizing a set of particles.
void
ParticleFilter::setVisualizer(const boost::shared_ptr<Visualizer>& vis)
{
	// Visualizer for the set of particles.
	p_visualizer = vis;
}


