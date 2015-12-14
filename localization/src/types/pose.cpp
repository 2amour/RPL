/*
 * pose.cpp
 *
 *	Pose types.
 *  Created on: Dec 5, 2015
 *      Author: Toni Rosinol
 */

#include "types/pose.h"

// Create empty pose.
template <typename T>
Pose<T>::Pose ()
{
	position.push_back(0); ///< x
	position.push_back(0); ///< y
	position.push_back(0); ///< z
	orientation.push_back(0); ///< yaw
	orientation.push_back(0); ///< pitch
	orientation.push_back(0); ///< roll
}

// Create 2D pose.
template <typename T>
Pose<T>::Pose (T position2D[2], T theta)
{
	position.push_back(position2D[0]); ///< x
	position.push_back(position2D[1]); ///< y
	orientation.push_back(theta); ///< theta
}

// Create 3D pose.
template <typename T>
Pose<T>::Pose (T position3D[3], T euler_angles[3])
{
	position.push_back(position3D[0]); ///< x
	position.push_back(position3D[1]); ///< y
	position.push_back(position3D[2]); ///< z

	orientation.push_back(euler_angles[0]); ///< yaw
	orientation.push_back(euler_angles[1]); ///< pitch
	orientation.push_back(euler_angles[2]); ///< roll
}

template class Pose<float>;
template class Pose<double>;

