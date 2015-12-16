/*
 * sensor_updater.cpp
 *
 *	Sensor updater algorithm.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#include "algorithms/sensor_updater/sensor_updater.h"

// Virtual destructor.
SensorUpdater::~SensorUpdater ()
{
}

// Get cumulated weight.
float
SensorUpdater::getCumulatedWeight ()
{
	return cumulated_weight;
}

// Get cumulated_weight.
void
SensorUpdater::setCumulatedWeight (float cw)
{
	cumulated_weight = cw;
}

// Map matching constructor.
MapMatching::MapMatching (const MapMatchingParams& params_input)
{
	// Initialize map properties.
	global_width = 0;
	global_height = 0;
	global_resolution = 0;

	// Initialize cumulated weight of particles.
	cumulated_weight = 0;

	// Initialize previous pose.
	past_pose.position[0] = 0;
	past_pose.position[1] = 0;
	past_pose.orientation[0] = 0;
}

// Map matching destructor.
MapMatching::~MapMatching ()
{
}

// Sensor update using map matching.
float
MapMatching::sensorUpdate (const MsgLaserScan::ConstPtr& z_t, const Pose<float>& current_pose)
{
	// Get detected points by sensor in current Pose frame.
	float curr_x = current_pose.position[0];
	float curr_y = current_pose.position[1];
	float theta = current_pose.orientation[0];

	float correlation = 0; ///< Correlation between real map and sensed map.
	int matches_counter = 0; ///< Counts the number of matches between sensor input and map.
	int valid_points_size = 0; ///< Counts the number of valid points sensed by the sensor.

	// If current particle falls inside the map and the resolution of the map is not zero, then compute correlation.
	if (curr_x > 0 && curr_x < global_width && curr_y > 0 && curr_y < global_height && global_resolution !=0)
		{
			// Sensor properties.
			float a_max =  z_t->angle_max;
			float a_min = z_t->angle_min;
			float delta = z_t->angle_increment;
			float r_max = z_t->range_max;
			float r_min = z_t->range_min;

			// Map index.
			int i_map = 0;
			int j_map = 0;

			// Range and angles sensed.
			float tmp_range = 0;
			float tmp_angle = a_min;

			// Sensed point in local coordinates.
			Point2D<float> point;

			// Sensed point in global coordinates.
			Point2D<float> global_point;

			// For all the sensed values.
			for (int i = 0; i < z_t->ranges.size(); ++i)
				{
					tmp_range = z_t->ranges[i];

					// Check whether we are sensing a valid point.
					if (tmp_range < r_max && tmp_range > r_min)
						{
							// Amount of valid points.
							++valid_points_size;

							// Converting sensor input to cartesian coordinates.
							point = polar2Cartesian (tmp_range, tmp_angle);

							// Converting point to global frame.
							global_point.x = point.x*cosf(theta) - point.y*sinf(theta) + curr_x;
							global_point.y = point.x*sinf(theta) + point.y*cosf(theta) + curr_y;

							// Corresponding indices in global map.
							i_map = (int)(global_point.x/global_resolution);
							j_map = (int)(global_point.y/global_resolution);

							// Check if the point is inside the global map.
							if (i_map >= 0 && i_map < global_width && j_map >= 0 && j_map < global_height)
								{
									// Check if this point is really an obstacle in the map (supposing that the robot is at this current particle).
									if (global_map->data[i_map+j_map*global_width]>0)
										{
											++matches_counter;
										}
								}
						}
					// Get next sensed angle.
					tmp_angle += delta;
				}
		}
	else
		{
			correlation = 0;
		}

	// Compute correlation.
	if (valid_points_size != 0 && matches_counter !=0)
		{
			correlation = (float)matches_counter/valid_points_size;
		}
	else
		{
			correlation = 0;
		}

	// Compute cumulated weight of the particles.
	cumulated_weight += correlation;

	// Return computed correlation for current pose.
	return correlation;
}

// Set input map.
void
MapMatching::setMap (const MsgMap::ConstPtr& map_input)
{
	// Store map properties.
	global_width = map_input->info.width;
	global_height = map_input->info.height;
	global_resolution = map_input->info.resolution;

	//Store global map.
	global_map = map_input;
}



