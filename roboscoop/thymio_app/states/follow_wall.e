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
			create orientation_controller.make_with_gains (0.6, 0.1, 0.1) -- TODO - HARDCODED
			create speed_controller.make_with_speed (0.02)
			create time_handler.start (0.0)
			create last_point.make
		ensure
			set_v: v_leave = v
		end

feature -- Access

	update_velocity (drive: separate DIFFERENTIAL_DRIVE)
		do
			drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
--			io.put_string ("Theta:" + orientation_controller.get_output.out + "%N")
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		do
			leds.set_to_red
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
			follow_wall: LINE_2D
			world_coordinate: TRANSFORM_2D
		do
			if range_signaler.sensors[1].is_valid_range then
				io.put_string (last_point.get_string + ", x: " + t_sig.get_pose.get_position.get_x.out + " y: " + t_sig.get_pose.get_position.get_y.out + "%N")
				create world_coordinate.make_with_offsets (t_sig.get_pose.get_position.get_x, t_sig.get_pose.get_position.get_y, t_sig.get_pose.get_orientation)
				last_point := world_coordinate.project_to_parent (create {POINT_2D}.make_with_coordinates (range_signaler.sensor_range_point (1).get_x, range_signaler.sensor_range_point (1).get_y))
				error := range_signaler.follow_wall_orientation (0.15) -- t_sig.get_pose.get_orientation -- TODO - HARDCODED
			else
				if t_sig.get_pose.get_position.get_euclidean_distance (last_point) < 0.15 then
--					io.put_string ("ROTATE")
--					create follow_wall.make_with_points (last_point, last_point + create {POINT_2D}.make_with_coordinates ({DOUBLE_MATH}.sine (t_sig.get_pose.get_orientation),
--																															{DOUBLE_MATH}.cosine (t_sig.get_pose.get_orientation)))
					create follow_wall.make_with_points (create {POINT_2D}.make_with_coordinates (0.0, 0.0),
															create {POINT_2D}.make_with_coordinates (0.0, 1.0))
					error := test (0.15, follow_wall)
				else
--					io.put_string ("STRAIGHT")
					create follow_wall.make_with_points (create {POINT_2D}.make_with_coordinates (0.0, 0.0),
															create {POINT_2D}.make_with_coordinates (1.0, 1.0))
					error := test (0.15, follow_wall)
				end
			end
--			io.put_string (" Distance: " + t_sig.get_pose.get_position.get_euclidean_distance (last_point).out + "%N")

			time_handler.set_time (t_sig.get_timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end

	test (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction.
		local
			current_distance: REAL_64
			v_wall: VECTOR_2D
			v_robot: VECTOR_2D
			v_theta: VECTOR_2D
		do
			current_distance := line.get_distance_from_point (create {POINT_2D}.make_with_coordinates (0.0, 0.0))
			v_wall := line.get_vector.get_unitary
			v_robot := v_wall.get_perpendicular

			v_theta := (v_wall*desired_distance) + (v_robot*(+(current_distance - desired_distance))) -- TODO - Change to minus (-) for follow obstacle in the right.
--			io.put_string ("x: " + v_theta.get_x.out + " y: " + v_theta.get_y.out + "%N")

			Result := v_theta.get_angle
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


		set_clockwise
				-- Set clockwise wall-following
			do
				clockwise := True
			end

		set_counter_clockwise
			do
				clockwise := False
			end

feature {NONE} -- Implementation

	clockwise: BOOLEAN
			-- Follow Wall in clockwise(True) or counter-clockwise(False) direction

	last_point: POINT_2D
			-- Last point detected in the wall the robot is following.

	v_leave : REAL_64
			-- Leave obstacle velocity.

	orientation_controller: PID
			-- Controller for the robot orientation.

	speed_controller: NON_LINEAR_SPEED
			-- Controller for the robot linear speed.

	time_handler: TIME_PARSER
			-- ROS message time stamp parser.
end
