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

	make_with_attributes (pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS)
		-- Make
		do
			create pose_controller.make_with_attributes (pose_controller_parameters.pid_parameters, pose_controller_parameters.nlsc_parameters,
															pose_controller_parameters.turning_angular_speed, pose_controller_parameters.reached_point_threshold,
															pose_controller_parameters.reached_orientation_threshold)
		end

feature -- Access

	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			pose_controller.update_drive_velocity (drive)
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		do
			pose_controller.set_current_pose (t_sig.current_pose, t_sig.timestamp)
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if r_sig.is_front_obstacle_close then
					if r_sig.is_obstacle_mostly_at_left then
						t_sig.set_follow_wall_counter_clockwise
					else
						t_sig.set_follow_wall_clockwise
					end
			elseif t_sig.goal.get_position.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.min_distance and
				pose_controller.is_target_position_reached then
				t_sig.set_go_to_goal
			end
		end

	set_safe_sensed_pose (leave_wall_safe_sensed_pose: separate POSE_2D)
			-- Setter for `safe_sensed_point'.
		do
			pose_controller.set_target_pose (leave_wall_safe_sensed_pose)
		end

feature {NONE} -- Implementation

	pose_controller: POSE_CONTROLLER
			-- Pose controller.
end
