/*
 * localization_sequencer.cpp
 *
 *	Sequencer for the localization process.
 *  Created on: Dec 4, 2015
 *      Author: Toni Rosinol
 */

#include "sequencer/localization_sequencer.h"

// Change ROS odometry message to a pose type.
void
odometry2Posef (const MsgOdometry::ConstPtr& motion, boost::shared_ptr<Pose<float> >& pose)
{
	// Position and orientation of the pose message.
	float position [3], orientation [3];

	// Extract angles from odometry message.
	tf::Quaternion q;
	q.setValue(motion->pose.pose.orientation.x, motion->pose.pose.orientation.y, motion->pose.pose.orientation.z, motion->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double yaw, pitch, roll;
	m.getEulerYPR(yaw, pitch, roll);

	// Create Pose from odometry message.
	position [0] = motion->pose.pose.position.x;
	position [1] = motion->pose.pose.position.y;
	position [2] = motion->pose.pose.position.z;
	orientation[0] = (float)yaw;
	orientation[1] = (float)pitch;
	orientation[2] = (float)roll;

	// Create Pose.
	boost::shared_ptr<Pose<float> > ptr_x_t (new Pose<float>(position, orientation));

	pose = ptr_x_t;
}

// Localization sequencer constructor, initializes algorithm, current particle, sensor input and map_ready flag.
LocalizationSequencer::LocalizationSequencer (const LocalizerParameters& params_bag):
	algorithm (params_bag.algorithm), x_t(), m_t(), z_t(), u_t(new Control<float>), localization(new Pose<float>),
	localization_pub(params_bag.pose_publisher), flag_pub (params_bag.flag_publisher), is_on(false), isFirstTimeLocalized(false)
{
	// Initialize flag.
	map_ready = false;

 	// Sets a visualizer for the algorithm output.
	algorithm->setVisualizer(params_bag.visualizer);
}

// Localization sequencer destructor.
LocalizationSequencer::~LocalizationSequencer ()
{
}

// Localization sequencer start.
void
LocalizationSequencer::start ()
{
	// Start localization.
	if (algorithm->localize(x_t, z_t, localization))
		{
			localization_pub->publishBelievedOdometry(localization, m_t);
			isFirstTimeLocalized = true;
			isLocalized.data = true;
			flag_pub.publish(isLocalized);
		}
	else
		{
			isLocalized.data = false;
			flag_pub.publish(isLocalized);
		}
}

// Publishes believed odometry or robot odometry depending on whether we are localized or not.
void
LocalizationSequencer::updateOdometry (const MsgOdometry::ConstPtr& motion)
{
	std::cout << "Odometry" << std::endl;
	// Create Pose.
	boost::shared_ptr<Pose<float> > ptr_x_t (new Pose<float>());

	// Change message to Pose<float> type.
	odometry2Posef (motion, ptr_x_t);
	m_t = motion;

	// Store odometry.
	u_t->array[0] = u_t->array[1]; ///< Store odometry at time t-1.
	u_t->array[1] = *ptr_x_t; ///< Store odometry at time t.

	// Update pose with input of the robot.
	updatePose(*u_t, *localization);

	// Publish TF from odometry link to map.
	localization_pub->publishTF(localization, motion);

	// Publishes localization odometry of the robot if first time localized even if afterwards we lose localization.
	if (isFirstTimeLocalized)
		{
			// Publish believed odometry of the robot.
			localization_pub->publishBelievedOdometry(localization, motion);
		}
	else
		{
			// Publish real odometry of the robot.
			localization_pub->publishOdometry(motion);
		}
}

// Localization sequencer send map to algorithm.
void
LocalizationSequencer::updatedMap (const MsgMap::ConstPtr& map_input)
{
	// Set map to ready.
	map_ready = true;

	// Set map for algorithm.
	algorithm->setMap(map_input);
}

// Update from scan msg.
void
LocalizationSequencer::updateScan (const MsgLaserScan::ConstPtr& sensor)
{
	std::cout << "scan" << std::endl;
		// Check whether it is requested to localize.
		if (is_on)
			{
				// Create Pose.
				boost::shared_ptr<Pose<float> > ptr_x_t (new Pose<float>());

				// Change message to Pose type.
				odometry2Posef (m_t, ptr_x_t);

				// Store input.
				x_t = ptr_x_t; ///< Store pose at time t.

				z_t = sensor; ///< Store sensor msg at time t.

				u_t->array[0] = u_t->array[1]; ///< Store pose at time t-1.
				u_t->array[1] = *x_t; ///< Store pose at time t.

				// If map is set then start localization.
				if (map_ready)
					{
						start ();
					}
			}
		else
			{
				//algorithm->reset();
			}
}


// On off command for localization.
void
LocalizationSequencer::onOffCallback (const onOff& flag_on_off)
{
	is_on = flag_on_off->data;
}

// Update localization pose by applying a motion update without noise.
void
LocalizationSequencer::updatePose (const Control<float>& control, Pose<float>& initial_pose)
{
		float rot1, trans, rot2;

		// Input pose at time t.
		float x1 = control.array[1].position[0];
		float y1 = control.array[1].position[1];
		float theta1 = control.array[1].orientation[0];

		// Input pose at time t-1.
		float x0 = control.array[0].position[0];
		float y0 = control.array[0].position[1];
		float theta0 = control.array[0].orientation[0];

		// Initial pose.
		float x = initial_pose.position[0];
		float y = initial_pose.position[1];
		float theta = initial_pose.orientation[0];

		double diffx = x1-x0;
		double diffy = y1-y0;

		// Recover relative motion parameters from robot control.
		rot1 = atan2(diffy, diffx) - theta0;
		trans = sqrt(pow(diffx,2) + pow(diffy,2));
		rot2 = theta1 - theta0 - rot1;

		// Update the output pose.
		float x_, y_, theta_;
		x_ = x + trans*cosf(theta + rot1);
		y_ = y + trans*sinf(theta + rot1);
		theta_ = theta + rot1 + rot2;

		// Store result.
		float position [2] = {x_, y_};
		float orientation [1] = {theta_};
		Pose<float> result (position, orientation);

		initial_pose = result;
}


