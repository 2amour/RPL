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
ParticlesVisualizer::publish (const std::vector<Particle>& particles)
{
	// Create ROS msg.
	geometry_msgs::PoseArray arr;
	geometry_msgs::Pose pose;
	tf::Quaternion q;

	// Set header information of the message.
	arr.header.frame_id = "map";
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


// Constructor.
LocalizationPublisher::LocalizationPublisher (std::string f_id)
{
	// Frame id for the published message header.
	frame_id = f_id;
}

// Destructor.
LocalizationPublisher::~LocalizationPublisher ()
{
}

// Set publisher.
void
LocalizationPublisher::setPublisher (const ros::Publisher p)
{
	// Set publisher.
	pub = p;
}

// Publish particles.
void
LocalizationPublisher::publish (const boost::shared_ptr<Pose<float> >& localization, const MsgOdometry::ConstPtr& m_t)
{
	// Create pose ROS msg.
	geometry_msgs::Pose pose;
	tf::Quaternion q;

	// Make pose position.
	pose.position.x = localization->position[0];
	pose.position.y = localization->position[1];
	pose.position.z = 0;

	// Make pose orientation.
	q.setEulerZYX(localization->orientation[0], 0, 0);
	tf::quaternionTFToMsg(q,pose.orientation);

	// Publishing odometry of the location of the robot.
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "map";

	// Set the position.
	odom.pose.pose = pose;

	// Set the velocity.
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = m_t->twist.twist.linear.x;
	odom.twist.twist.linear.y = m_t->twist.twist.linear.y;
	odom.twist.twist.angular.z = m_t->twist.twist.angular.z;

	// Publish msg.
	pub.publish (odom);
}

// Publishes a transformation frame between the base link and map.
void
LocalizationPublisher::publishTF(const boost::shared_ptr<Pose<float> >& localization, const MsgOdometry::ConstPtr& m_t)
{
  tf::StampedTransform transform;

  // Extract position from localization, map frame.
  float x, y;
  x = localization->position[0];
  y = localization->position[1];

  // Extract orientation from localization, map frame.
  float theta = localization->orientation[0];

  // Extract position from odometry message, odometry_link frame.
  float odo_x = m_t->pose.pose.position.x;
  float odo_y = m_t->pose.pose.position.y;

	// Extract angles from odometry message, odometry_link frame.
	tf::Quaternion a;
	a.setValue(m_t->pose.pose.orientation.x, m_t->pose.pose.orientation.y, m_t->pose.pose.orientation.z, m_t->pose.pose.orientation.w);
	tf::Matrix3x3 m(a);
	double yaw, pitch, roll;
	m.getEulerYPR(yaw, pitch, roll);

	tf::Vector3 p (x, y, 0);
	tf::Quaternion q;

	// Converting odometry message to map frame orientation without translation to map frame.
	float g_odo_x, g_odo_y;
  g_odo_x = odo_x*cosf(-yaw + theta) - odo_y*sinf(-yaw + theta);
  g_odo_y = odo_x*sinf(-yaw + theta) + odo_y*cosf(-yaw + theta);

  // Make origin of transform.
  p.setX(x - g_odo_x);
  p.setY(y - g_odo_y);

  // Make quaternion of transform by adding angles.
   q.setEulerZYX(-yaw + theta, 0, 0);

  // Creating the transform message.
  transform.setOrigin(p);
  transform.setRotation(q);
  transform.child_frame_id_ = m_t->header.frame_id;
  transform.frame_id_ = "map";
  transform.stamp_ = m_t->header.stamp;

  _transform_broadcaster.sendTransform(transform);


}


