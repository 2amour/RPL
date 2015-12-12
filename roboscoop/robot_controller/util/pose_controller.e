note
	description: "Controller for the robot to reach a certain pose."
	author: "Ferran Pallarès"
	date: "11.12.2015"

class
	POSE_CONTROLLER

create make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (pid_parameters: separate PID_PARAMETERS; nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS;
							a_turning_angular_speed, a_reached_point_threshold, a_reached_orientation_threshold: REAL_64)
			-- Create self with given attributes.
		do
			create target_pose.make
			create current_pose.make
			create orientation_controller.make_with_gains (pid_parameters.kp, pid_parameters.ki, pid_parameters.kd)
			create speed_controller.make_with_attributes (nlsc_parameters.maximum_speed, nlsc_parameters.angular_decay_rate)
			create time_handler.start (0.0)

			turning_angular_speed := a_turning_angular_speed
			reached_point_threshold := a_reached_point_threshold
			reached_orientation_threshold := a_reached_orientation_threshold

			create math
		end

feature -- Access

	current_pose: POSE_2D
			-- Current robot pose.

	target_pose: POSE_2D
			-- Desired pose to reach.

	turning_angular_speed: REAL_64
			-- Desired angular speed for orienting the robot once the desired point is reached.

	reached_point_threshold: REAL_64
			-- Threshold for consider whether a given point has been reached or not.

	reached_orientation_threshold: REAL_64
			-- Threshold for consider whether a given orientation has been reached or not.

	set_current_pose (a_current_pose: separate POSE_2D; a_current_pose_timestamp: REAL_64)
			-- Set `current_pose' and compute the velocity command.
		do
			create current_pose.make_from_separate (a_current_pose)
			compute_velocity_command (current_pose, a_current_pose_timestamp)
		end

	set_target_pose (a_target_pose: separate POSE_2D)
			-- Setter for `target_pose'.
		do
			target_pose := create {POSE_2D}.make_from_separate (a_target_pose)
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

	update_drive_velocity (drive: separate DIFFERENTIAL_DRIVE)
			-- Assign velocity command to drive.
		do
			set_drive_velocity (drive, linear_speed_command, angular_speed_command)
		end


	is_target_position_reached: BOOLEAN
			-- Whether the target position has been reached.

	is_target_pose_reached: BOOLEAN
			-- Whether the target pose has been reached.

feature {NONE} -- Implementation

	orientation_controller: PID_CONTROLLER
			-- Controller for the robot orientation while moving towards the desired point.

	speed_controller: NON_LINEAR_SPEED_CONTROLLER
			-- Controller for the robot speed while moving towards the desired point.

	time_handler: TIME_HANDLER
			-- Time stamp manager.

	math: TRIGONOMETRY_MATH
			-- Math object.

	linear_speed_command: REAL_64
			-- Calculated linear speed to move towards the target.

	angular_speed_command: REAL_64
			-- Calculated angular speed to move towards the target.

	compute_velocity_command (a_current_pose: separate POSE_2D; a_current_pose_timestamp: REAL_64)
			-- Compute required velocity command in order to reach the desired pose
		local
			current_point: POINT_2D
			current_orientation: REAL_64
		do
			create current_point.make_from_separate (a_current_pose.get_position)
			current_orientation := a_current_pose.get_orientation

			time_handler.set_time (a_current_pose_timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)

				if current_point.get_euclidean_distance (target_pose.get_position) > reached_point_threshold then
					is_target_position_reached := False
					is_target_pose_reached := False
					go_to_target (current_point, target_pose.get_position, current_orientation)
				elseif {DOUBLE_MATH}.dabs (current_orientation - target_pose.get_orientation) > reached_orientation_threshold then
					is_target_position_reached := True
					is_target_pose_reached := False
					rotate (current_orientation, target_pose.get_orientation)
				else
					is_target_position_reached := True
					is_target_pose_reached := True
					linear_speed_command := 0.0
					angular_speed_command := 0.0
				end
			end
		end

	go_to_target (a_current_point, a_target_point: separate POINT_2D; a_current_orientation: REAL_64)
			-- Proceed to move towards the target.
		local
			error: REAL_64
		do
			if a_target_point.get_y - a_current_point.get_y = 0 and
			   a_target_point.get_x - a_current_point.get_x = 0 then
				error := 0
			end
			error := math.atan2 (a_target_point.get_y - a_current_point.get_y, a_target_point.get_x - a_current_point.get_x) - a_current_orientation
			error := math.atan2 (math.sine (error), math.cosine (error))

			orientation_controller.set_error (error)
			angular_speed_command := orientation_controller.get_output

			speed_controller.set_angular_velocity (angular_speed_command)
			linear_speed_command := speed_controller.get_output
		end

	rotate (a_current_orientation, a_target_orientation: REAL_64)
			-- Proceed to rotate to reach the target orientation.
		do
			linear_speed_command := 0.0

			if a_current_orientation > a_target_orientation then
				angular_speed_command := -turning_angular_speed
			else
				angular_speed_command := turning_angular_speed
			end
		end

	set_drive_velocity (a_drive: separate DIFFERENTIAL_DRIVE; a_linear_speed_command, a_angular_speed_command: REAL_64)
			-- Update robot drive given the desired velocity.
		do
			a_drive.set_velocity (a_linear_speed_command, a_angular_speed_command)
		end
end
