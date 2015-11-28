note
    description: "In this state the robot is heading directly towards the goal."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	GO_TO_GOAL

inherit
	TANGENT_BUG_STATE

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (pid_parameters: separate PID_PARAMETERS; nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS)
			-- Create self with attributes.
		do
			create math
			create time_handler.start (0.0)
			create speed_controller.make_with_attributes (0.08, 2)
			create orientation_controller.make_with_gains (pid_parameters.kp, pid_parameters.ki, pid_parameters.kd)
		end

feature

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

			if t_sig.goal.get_y - t_sig.current_pose.get_position.get_y = 0 and
			   t_sig.goal.get_x - t_sig.current_pose.get_position.get_x = 0 then
				error := 0
			else
				error := math.atan2(t_sig.goal.get_y - t_sig.current_pose.get_position.get_y, t_sig.goal.get_x - t_sig.current_pose.get_position.get_x) - t_sig.current_pose.get_orientation
				error := math.atan2 (math.sine (error), math.cosine (error))
			end

			time_handler.set_time(t_sig.timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if r_sig.is_obstacle_in_front then
				if r_sig.is_obstacle_mostly_at_left then
					t_sig.set_follow_wall_counter_clockwise
				else
					t_sig.set_follow_wall_clockwise
				end
			end
			if t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.goal_threshold then
				t_sig.set_at_goal
			end
		end

feature {NONE} -- Implementation

	orientation_controller: PID_CONTROLLER

	speed_controller: NON_LINEAR_SPEED_CONTROLLER

	time_handler: TIME_HANDLER

	math: TRIGONOMETRY_MATH
end
