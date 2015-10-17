note
	description: "In this state the robot follows the perimeter of an obstacle."
	author: "Ferran Pallarès"
	date: "16.10.2015"

class
	FOLLOW_WALL

inherit
	STATE

create
	make_with_v_leave

feature -- Initialization

	make_with_v_leave (v: REAL_64)
			-- Create self.
		require
			positive_v: v > 0
		do
			v_leave := v
			create orientation_controller.make_with_gains (0.6, 0.0, 0.0) -- TODO - HARDCODED
			create speed_controller.make
			create time_handler.start (0.0)
		ensure
			set_v: v_leave = v
		end

feature -- Access

	update_velocity (drive: separate DIFFERENTIAL_DRIVE)
		do
			drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		do
			leds.set_to_red
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
		do
			error := range_signaler.follow_wall_orientation (10) - t_sig.get_pose.get_orientation -- TODO - HARDCODED
			time_handler.set_time (t_sig.get_timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		local
			v_leave_point: separate POINT_2D
			vector_to_goal: VECTOR_2D
		do
			create vector_to_goal.make_from_points(create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y), t_sig.get_goal)

			create v_leave_point.make_with_coordinates (o_sig.x + v_leave * {DOUBLE_MATH}.cosine(vector_to_goal.get_angle),
														o_sig.y + v_leave * {DOUBLE_MATH}.sine  (vector_to_goal.get_angle))

			if r_sig.has_obstacle (vector_to_goal.get_angle) and
				t_sig.get_goal.get_euclidean_distance ( v_leave_point ) < t_sig.get_d_min then
				t_sig.set_leave_wall
			end
		end

feature {NONE} -- Implementation

	v_leave : REAL_64
			-- Leave obstacle velocity.

	orientation_controller: PID
			-- Controller for the robot orientation.

	speed_controller: NON_LINEAR_SPEED
			-- Controller for the robot linear speed.

	time_handler: TIME_PARSER
			-- ROS message time stamp parser.
end
