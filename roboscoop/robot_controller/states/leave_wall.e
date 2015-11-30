note
	description: "In this state the robot heads a safe point closer to the goal than has ever been."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	LEAVE_WALL

inherit
	TANGENT_BUG_STATE

create
	make_with_attributes

feature -- Initialization

	make_with_attributes (pid_parameters: separate PID_PARAMETERS; nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS)
		-- Make
		do
			create math
			create time_handler.start (0.0)
			create speed_controller.make_with_attributes (nlsc_parameters.maximum_speed, nlsc_parameters.angular_decay_rate)
			create orientation_controller.make_with_gains (pid_parameters.kp, pid_parameters.ki, pid_parameters.kd)
			create safe_sensed_point.make
		end

feature -- Access

	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		local
			error: REAL_64
		do

			error := math.atan2 (safe_sensed_point.get_y - t_sig.current_pose.get_position.get_y, safe_sensed_point.get_x - t_sig.current_pose.get_position.get_x) - t_sig.current_pose.get_orientation
			error := math.atan2 (math.sine (error), math.cosine (error))
			time_handler.set_time (t_sig.timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		local
			current_point: POINT_2D
		do
			if r_sig.is_front_obstacle_close then
					t_sig.set_intial_point_wall (t_sig.current_pose.get_position)
					t_sig.set_initial_orientation (t_sig.current_pose.get_orientation)
					
					if r_sig.is_obstacle_mostly_at_left then
						t_sig.set_follow_wall_counter_clockwise
					else
						t_sig.set_follow_wall_clockwise
					end
			elseif t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.min_distance and
				safe_sensed_point.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.reached_point_threshold then
				t_sig.set_go_to_goal
			end
		end

	set_safe_sensed_point (leave_wall_safe_sensed_point: separate POINT_2D)
			-- Setter for `safe_sensed_point'.
		do
			safe_sensed_point := create {POINT_2D}.make_from_separate (leave_wall_safe_sensed_point)
		ensure
			target_set: safe_sensed_point = leave_wall_safe_sensed_point
		end

feature {NONE} -- Implementation

	safe_sensed_point: POINT_2D
			-- Safe sensed point closer than the robot has ever been.

	orientation_controller: PID_CONTROLLER
			-- Orientation controller.

	speed_controller: NON_LINEAR_SPEED_CONTROLLER
			-- Speed controller.

	time_handler: TIME_HANDLER
			-- Object for time stamp managing.

	math: TRIGONOMETRY_MATH
			-- Math object.
end
