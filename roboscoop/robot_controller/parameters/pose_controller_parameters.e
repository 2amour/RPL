note
	description: "Pose controller parameters."
	author: "Ferran Pallarès"
	date: "11.12.2015"

class
	POSE_CONTROLLER_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty pose controller parameters object.
		do
			create pid_parameters.make
			create nlsc_parameters.make
		end

	make_with_attributes (a_pid_parameters: separate PID_PARAMETERS; a_nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS; a_turning_angular_speed, a_reached_point_threshold, a_reached_orientation_threshold: REAL_64)
			-- Create pose controller parameters object with attributes.
		do
			create pid_parameters.make_from_separate (a_pid_parameters)
			create nlsc_parameters.make_from_separate (a_nlsc_parameters)
			turning_angular_speed := a_turning_angular_speed
			reached_point_threshold := a_reached_point_threshold
			reached_orientation_threshold := a_reached_orientation_threshold
		end

	make_from_separate (other: separate POSE_CONTROLLER_PARAMETERS)
			-- Create pose controller parameters object from separate other.
		do
			create pid_parameters.make_from_separate (other.pid_parameters)
			create nlsc_parameters.make_from_separate (other.nlsc_parameters)
			turning_angular_speed := other.turning_angular_speed
			reached_point_threshold := other.reached_point_threshold
			reached_orientation_threshold := other.reached_point_threshold
		end

feature -- Access

	pid_parameters: PID_PARAMETERS
			-- PID parameters.

	nlsc_parameters: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			-- Non-linear speed controller parameters.

	turning_angular_speed: REAL_64
			-- Desired angular speed for orienting the robot once the desired point is reached.

	reached_point_threshold: REAL_64
			-- Threshold for consider whether a given point has been reached or not.

	reached_orientation_threshold: REAL_64
			-- Threshold for consider whether a given orientation has been reached or not.

	set_pid_parameters (a_pid_parameters: separate PID_PARAMETERS)
			-- Setter for `pid_parameters'.
		do
			create pid_parameters.make_from_separate (a_pid_parameters)
		end

	set_nlsc_parameters (a_nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS)
			-- Setter for `nlsc_parameters'.
		do
			create nlsc_parameters.make_from_separate (a_nlsc_parameters)
		end

	set_turning_angular_speed (a_turning_angular_speed: REAL_64)
			-- Setter for `turning_angular_speed'.
		do
			turning_angular_speed := a_turning_angular_speed
		end

	set_reached_point_threshold (a_reached_point_threshold: REAL_64)
			-- Setter for `reached_point_threshold'.
		do
			reached_point_threshold := a_reached_point_threshold
		end

	set_reached_orientation_threshold (a_reached_orientation_threshold: REAL_64)
			-- Setter for `reached_orientation_threshold'.
		do
			reached_orientation_threshold := a_reached_orientation_threshold
		end
end
