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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "types/pose.h"
#include "tf/tf.h"

typedef nav_msgs::Odometry MsgOdometry; ///< Message for odometry.

// Deferred class for data publishers.
class Visualizer {
public:
	// Virtualizer destructor.
	virtual ~Visualizer ();

	// Set publisher.
	virtual void setPublisher (const ros::Publisher p) = 0;

	// Publish PoseArray.
	virtual void publish (const std::vector<Particle>& particles) = 0;

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
	void publish (const std::vector<Particle>& particles);

private:

	// Frame id for message header.
	std::string frame_id;

	// Publisher.
	ros::Publisher pub;
};

// Localization publisher.
class LocalizationPublisher
{
public:
	// Constructor.
	LocalizationPublisher (std::string f_id);

	// Destructor.
	virtual ~LocalizationPublisher ();

	// Sets the ROS publisher.
	void setPublisher (const ros::Publisher p);

	// Publishes the believed pose of the robot.
	void publishBelievedOdometry(const boost::shared_ptr<Pose<float> >& position, const MsgOdometry::ConstPtr& m_t);

	// Publishes odometry of the robot with respect to last localization.
	void publishOdometry(const MsgOdometry::ConstPtr& m_t);

	// Publishes a transform with the location information.
	void publishTF(const boost::shared_ptr<Pose<float> >& localization, const MsgOdometry::ConstPtr& m_t);

private:

	// Transform broadcaster for changing the odometry link frame of the robot to the believed location.
	tf::TransformBroadcaster  _transform_broadcaster;

	// Frame id for message header.
	std::string frame_id;

	// Publisher.
	ros::Publisher pub;
};

// Localization publisher
class FlagPublisher
{
public:
	// Constructor.
	FlagPublisher ();

	// Destructor.
	virtual ~FlagPublisher ();

	// Sets the ROS publisher.
	void setPublisher (const ros::Publisher p);

	// Publishes whether we are localized or not.
	void publish(bool flag);

private:

	// Publisher.
	ros::Publisher pub;
};

#endif /* LOCALIZATION_INCLUDE_VISUALIZER_VISUALIZER_H_ */
