/*
 * visualizer.h
 *
 *	Visualizer for publishing localization result.
 *  Created on: Dec 6, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_VISUALIZER_VISUALIZER_H_
#define LOCALIZATION_INCLUDE_VISUALIZER_VISUALIZER_H_

#include <ros/publisher.h>
#include <geometry_msgs/PoseArray.h>

#include "types/pose.h"
#include "tf/tf.h"

// Deferred class for data publishers.
class Visualizer {
public:
	// Virtualizer destructor.
	virtual ~Visualizer ();

	// Set publisher.
	virtual void setPublisher (const ros::Publisher p) = 0;

	// Publish particles.
	virtual void publishParticles (const std::vector<Particle>& particles) = 0;
};

// Particles specific visualizer
class ParticlesVisualizer: public Visualizer
{
public:
	// Constructor.
	ParticlesVisualizer (std::string f_id);

	// Destructor.
	virtual ~ParticlesVisualizer ();

	// Sets the ROS publisher.
	void setPublisher (const ros::Publisher p);

	// Publishes the particles.
	void publishParticles (const std::vector<Particle>& particles);

private:

	// Frame id for message header.
	std::string frame_id;

	// Publisher.
	ros::Publisher pub;
};

#endif /* LOCALIZATION_INCLUDE_VISUALIZER_VISUALIZER_H_ */
