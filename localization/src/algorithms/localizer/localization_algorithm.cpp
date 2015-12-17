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
		p_motion_updater(params.motion_updater), p_sensor_updater(params.sensor_updater), p_resampler (params.resampler),
		u_t(new Control<float>), x_t1(new Pose<float>), x_t0(new Pose<float>), local_map(), p_visualizer()
{
		// Initialize parameters.
		angle_threshold = params.angle_threshold;
		position_threshold = params.position_threshold;
		cw = 0;
		height = 0;
		width = 0;
		size = 0;
		resolution = 0;
		cells_jumped = params.cells_jumped;
		delta_theta = params.delta_theta;
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

	// Create initial particles in the map.
	for (int i = width/8; i < (int)3*width/8; i += cells_jumped)
		{
			for (int j = width/8; j < (int)3*height/8; j += cells_jumped)
				{
					if (map->data[i+j*(int)(3*width/8)]==0)
						{
							for (float theta = 0; theta < 2*M_PI; theta += delta_theta)
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
bool
ParticleFilter::localize(const boost::shared_ptr<Pose<float> >& x_t, const MsgLaserScan::ConstPtr& z_t, const boost::shared_ptr<Pose<float> >& localization)
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

			// Store cumulated weight.
			cw = p_sensor_updater->getCumulatedWeight();

			// Resampling.
			p_resampler->resample(particles_ptr, cw);

			// Reinitialization of cumulated_weight.
			p_sensor_updater->setCumulatedWeight(0);
		}

	// Visualize particles.
	p_visualizer->publish(*particles_ptr);

	// Return whether we are localized or not and store the believed localization.
	return getLocalization (localization);
}

// Check whether we are localized or not and set the localization pose.
bool
ParticleFilter::getLocalization(const boost::shared_ptr<Pose<float> >& localization)
{
	// Whether we are localized or not.
	bool result = false;

	// Current cumulated weight.
	float tmp_weight = 0;

	// Median pose.
	Pose<float> median;

	// Flag median found.
	bool isMedianFound;

	// Average values.
	float sum_theta, sum_x, sum_y = 0;
	float avg_theta, avg_x, avg_y = 0;

	// Compute the location of the robot by taking the average properties of the particles.
	for (int i = 0; i < particles_ptr->size(); ++i)
		{
			tmp_weight += particles_ptr->at(i).weight;
			if (tmp_weight > cw / 2 && !isMedianFound)
				{
					median = particles_ptr->at(i).pose;
					isMedianFound = true;
				}
			sum_theta += particles_ptr->at(i).pose.orientation[0];
			sum_x += particles_ptr->at(i).pose.position[0];
			sum_y += particles_ptr->at(i).pose.position[1];
		}

	avg_theta = sum_theta / particles_ptr->size();
	avg_x = sum_x / particles_ptr->size();
	avg_y = sum_y / particles_ptr->size();

	// Calculates standard deviation of particles poses.
	float dev_theta, dev_x, dev_y = 0;
	for (int i = 0; i < particles_ptr->size(); ++i)
		{
			dev_theta += pow(particles_ptr->at(i).pose.orientation[0]-avg_theta, 2);
			dev_x += pow(particles_ptr->at(i).pose.position[0]-avg_x, 2);
			dev_y += pow(particles_ptr->at(i).pose.position[1]-avg_y, 2);
		}

	dev_theta = sqrt(dev_theta/particles_ptr->size());
	dev_x = sqrt(dev_x/particles_ptr->size());
	dev_y = sqrt(dev_y/particles_ptr->size());

	// Checks whether we have a good estimation for the current pose.
	if (dev_theta < angle_threshold && dev_x < position_threshold && dev_y < position_threshold)
		{
			result = true;
			localization->orientation[0] = avg_theta;
			localization->position[0] = avg_x;
			localization->position[1] = avg_y;
		}

	// Sends whether we found a valid localization.
	return result;
}

// Saves messages of the sensor into a local map.
void
ParticleFilter::makeLocalMap(const MsgLaserScan::ConstPtr& sensor, const boost::shared_ptr<Pose<float> >& curr_pose)
{
	// Store data of current pose.
	float theta, curr_x, curr_y;
	theta = curr_pose->orientation[0];
	curr_x = curr_pose->position[0];
	curr_y = curr_pose->position[1];

	// Transform sensor polar points to cartesian points in sensor frame.
	// Sensor properties.
	float a_max =  sensor->angle_max;
	float a_min = sensor->angle_min;
	float delta = sensor->angle_increment;
	float r_max = sensor->range_max;
	float r_min = sensor->range_min;

	// Map index.
	int i_map = 0;
	int j_map = 0;

	// Range and angles sensed.
	float tmp_range = 0;
	float tmp_angle = a_min;
	float tmp_orientation = 0; // temporary variable for storing angle between pair of sensed points.
	float tmp_orientation_0 = 0; // Variable for storing previous tmp_orientation.
	boost::shared_ptr<Particle> tmp_particle (new Particle);

	// Hash code.
	float id;

	// Sensed point in local coordinates.
	Point2D<float> point;

	// Previous sensed point in local coordinates.
	Point2D<float> point_0;

	// Sensed point in global coordinates.
	Point2D<float> global_point;

	// Check that the orientation has changed.
	if (x_t1->orientation[0] != x_t0->orientation[0])
		{
			// For all the sensed values.
			for (int i = 0; i < sensor->ranges.size(); i += 1)
				{
					tmp_range = sensor->ranges[i];

					// Check whether we are sensing a valid point.
					if (tmp_range < r_max && tmp_range > r_min)
						{
							// Converting sensor input to cartesian coordinates.
							point = polar2Cartesian (tmp_range, tmp_angle);

							// Compute vector between current and past point.
							tmp_orientation = atan2 (point.y - point_0.y, point.x - point_0.x);

							// Check if we are sensing a straight wall.
							if (tmp_orientation != tmp_orientation_0)
								{
									// Converting point to global frame.
									global_point.x = point.x*cosf(theta) - point.y*sinf(theta) + curr_x;
									global_point.y = point.x*sinf(theta) + point.y*cosf(theta) + curr_y;

									// Make a particle.
									tmp_particle->pose.position[0] = global_point.x;
									tmp_particle->pose.position[1] = global_point.y;

									// Check that this point doesnt already exist in the local map.
									tmp_particle->pose.orientation [0] = tmp_orientation;
									local_map.push_back(*tmp_particle);
								}
						}
					// Store current point as previous.
					tmp_orientation_0 = tmp_orientation;
					point_0 = point;

					// Get next sensed angle.
					tmp_angle += delta;
				}
		}

	// Store current pose as past pose.
	x_t1 = x_t0;

	p_visualizer->publish(local_map);
}

// Reset algorithm.
void
ParticleFilter::reset ()
{
	// Reset values.
  *u_t = Control<float> ();
	*x_t1 = Pose<float> ();
	*x_t0 = Pose<float> ();

	// Get new particles from map.
	getParticlesFromMap ();
}

// Sets the visualizer for visualizing a set of particles.
void
ParticleFilter::setVisualizer(const boost::shared_ptr<Visualizer>& vis)
{
	// Visualizer for the set of particles.
	p_visualizer = vis;
}


