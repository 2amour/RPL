/*
 * visualizer.cpp
 *
 *	Visualizer for publishing localization result.
 *  Created on: Dec 6, 2015
 *      Author: toni
 */

#include "visualizer/visualizer.h"

// Visualizer virtual destructor.
Visualizer::~Visualizer ()
{
}

// Constructor.
ParticlesVisualizer::ParticlesVisualizer (std::string f_id)
{
	// Frame id for the published message header.
	frame_id = f_id;
}

// Destructor.
ParticlesVisualizer::~ParticlesVisualizer ()
{
}

// Set publisher.
void
ParticlesVisualizer::setPublisher (const ros::Publisher p)
{
	// Set publisher.
	pub = p;
}

// Publish particles.
void
ParticlesVisualizer::publishParticles (const std::vector<Particle>& particles)
{
	// Create ROS msg.
	geometry_msgs::PoseArray arr;
	geometry_msgs::Pose pose;
	tf::Quaternion q;

	// Set header information of the message.
	arr.header.frame_id = "odometry_link";
	arr.header.stamp = ros::Time::now();

	for (int i = 0; i < particles.size(); ++i)
		{
			// Make position part of ROS message.
			pose.position.x = particles[i].pose.position[0];
			pose.position.y = particles[i].pose.position[1];
			pose.position.z = particles[i].pose.position[2];

			// Make orientation part of ROS message.
			q.setEulerZYX(particles[i].pose.orientation[0], particles[i].pose.orientation[1], particles[i].pose.orientation[2]);
			tf::quaternionTFToMsg(q,pose.orientation);

			// Put the pose.
			arr.poses.push_back(pose);
		}

	// Publish msg.
	pub.publish (arr);
}
