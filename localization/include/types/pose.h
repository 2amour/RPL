/*
 * pose.h
 *
 *	Pose types.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#ifndef LOCALIZATION_INCLUDE_TYPES_POSE_H_
#define LOCALIZATION_INCLUDE_TYPES_POSE_H_

#include <vector>
#include <Eigen/Geometry>

// Generic Pose.
template <typename T>
class Pose
{
public:
	Pose ();
	Pose (T position2D[2], T theta);
	Pose (T position3D[3], T euler_angles [3]);

	std::vector<T> position;
	std::vector<T> orientation;
};

// Particle type.
struct Particle {
	Pose<float> pose;
	float weight;
};

// Pose in 2D.
struct Pose2D
{
	Eigen::Vector2f position;
	float angle;
};

// Pose in 3D.
struct Pose3D
{
	Eigen::Vector3f position;
	Eigen::Quaternionf orientation;
};

// Control type.
template <typename T>
struct Control
{
	Pose<T> array [2];
};

// Point2D.
template <typename T>
struct Point2D {
	T x;
	T y;
};

// Point3D.
template <typename T>
struct Point3D {
	T x;
	T y;
	T z;
};

// Orientation2D.
template <typename T>
struct Orientation2D {
	T theta;
};

// Orientation3D.
template <typename T>
struct Orientation3D {
	T yaw;
	T pitch;
	T roll;
};

#endif /* LOCALIZATION_INCLUDE_TYPES_POSE_H_ */
