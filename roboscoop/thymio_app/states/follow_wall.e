note
	description: "In this state the robot follows the perimeter of an obstacle."
	author: "Ferran Pallarès"
	date: "16.10.2015"

class
	FOLLOW_WALL

inherit
	STATE

create
	make

feature -- Initialization

	make
			-- Create self.
		do
			clockwise := False
			turning_angular_velocity := 0.2
			target_threshold := 0.10
			distance_from_wall := 0.15
			create corner_offset.make_with_coordinates (0.20, 0.03)

			create world_tf.make
			create target_point.make

			create orientation_controller.make_with_gains (0.6, 0.0, 0.2) -- TODO - HARDCODED
			create speed_controller.make_with_speed (0.02)
			create time_handler.start (0.0)
			create last_point.make
		end

feature -- Access

	update_velocity (drive: separate DIFFERENTIAL_DRIVE)
		do
			drive.set_velocity (linear_speed, angular_velocity)
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		do
			leds.set_to_red
		end

	set_readings (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			i: INTEGER_32
		do

			time_handler.set_time (t_sig.get_timestamp)
			orientation_controller.set_sampling (time_handler.get_sampling_rate)

			if time_handler.get_sampling_rate > 0 then
				if clockwise then
					if range_signaler.is_wall_only_at_right then
						follow_wall (t_sig, range_signaler)
					elseif range_signaler.is_huge_at_left then
						inner_corner (t_sig, range_signaler)
					elseif range_signaler.is_all_front_sensors_open then
						outer_corner (t_sig, range_signaler)
					end
				else
					if range_signaler.is_wall_only_at_left then
						follow_wall (t_sig, range_signaler)
					elseif range_signaler.is_huge_at_right then
						inner_corner (t_sig, range_signaler)
					elseif range_signaler.is_all_front_sensors_open then
						outer_corner (t_sig, range_signaler)
					end
				end

				update_minimum_distance_to_goal (t_sig)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		local
			v_leave_point: separate POINT_2D
			vector_to_goal: VECTOR_2D
		do
--			create vector_to_goal.make_from_points(create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y), t_sig.get_goal)

--			create v_leave_point.make_with_coordinates (o_sig.x + v_leave * {DOUBLE_MATH}.cosine(vector_to_goal.get_angle),
--														o_sig.y + v_leave * {DOUBLE_MATH}.sine  (vector_to_goal.get_angle))

--			if r_sig.has_obstacle (vector_to_goal.get_angle) and
--				t_sig.get_goal.get_euclidean_distance ( v_leave_point ) < t_sig.get_d_min then
--				t_sig.set_leave_wall
--			end
		end

	set_clockwise
			-- Set clockwise wall-following.
		do
			corner_offset.make_with_coordinates (0.20, -0.03)
			turning_angular_velocity := -0.4
			clockwise := True
		end

	set_counter_clockwise
			-- Set clockwise wall-following.
		do
			corner_offset.make_with_coordinates (0.20, 0.03)
			turning_angular_velocity := 0.4
			clockwise := False
		end

	follow_wall (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
		do
			world_tf.make_with_offsets (t_sig.get_pose.get_position.get_x, t_sig.get_pose.get_position.get_y, t_sig.get_pose.get_orientation)
			target_point := world_tf.project_to_parent (corner_offset)

			if clockwise then
				error := range_signaler.follow_right_wall (distance_from_wall)
			else
				error := range_signaler.follow_left_wall (distance_from_wall)
			end

			orientation_controller.set_error (error)
			angular_velocity := orientation_controller.get_output

			speed_controller.set_angular_velocity (angular_velocity)
			linear_speed := speed_controller.get_output

			debug
				io.put_string ("Follow Wall %N")
			end
		end

	inner_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
		do
			if clockwise then
				error := range_signaler.follow_closest_wall_cw (distance_from_wall)
			else
				error := range_signaler.follow_closest_wall_ccw (distance_from_wall)
			end

			orientation_controller.set_error (error)
			angular_velocity := orientation_controller.get_output
			speed_controller.set_angular_velocity (angular_velocity)
			linear_speed := speed_controller.get_output

			debug
				io.put_string ("Inner Corner %N")
			end
		end

	outer_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
			line: LINE_2D
			math: TRIGONOMETRY_MATH
		do
			create math
			if target_point.get_euclidean_distance (t_sig.get_pose.get_position) > target_threshold then
				error := math.atan2 (target_point.get_y - t_sig.get_pose.get_position.get_y, target_point.get_x - t_sig.get_pose.get_position.get_x)  - t_sig.get_pose.get_orientation
				error := math.atan2 (math.sine (error), math.cosine (error))
				orientation_controller.set_error (error)
				angular_velocity := orientation_controller.get_output
				debug
					io.put_string (target_point.get_euclidean_distance (t_sig.get_pose.get_position).out + "%N")
				end
			else
				angular_velocity := turning_angular_velocity
				debug
					io.put_string ("Turn %N")
				end
				speed_controller.set_angular_velocity (angular_velocity)
				linear_speed := speed_controller.get_output
			end

		end

	update_minimum_distance_to_goal (t_sig: separate TANGENT_BUG_SIGNALER)
			-- Check if current distance to goal is smaller than the minimum
		do
			if t_sig.get_goal.get_euclidean_distance (t_sig.get_pose.get_position) < t_sig.get_d_min then
				t_sig.set_d_min (t_sig.get_goal.get_euclidean_distance (t_sig.get_pose.get_position))
			end
		end



feature {NONE} -- Implementation
	turning_angular_velocity: REAL_64
			-- Angular velocity when its only turning.

	distance_from_wall: REAL_64
			-- Distance to follow wall

	corner_offset: POINT_2D
			-- Target offset when finding a corner.

	world_tf: TRANSFORM_2D
			-- Transformation from base to robot.

	target_point: POINT_2D
			-- Target point, offset from last detected point

	target_threshold: REAL_64
			-- Target threshold for controller

	clockwise: BOOLEAN
			-- Follow Wall in clockwise(True) or counter-clockwise(False) direction

	last_point: POINT_2D
			-- Last point detected in the wall the robot is following.

	orientation_controller: PID
			-- Controller for the robot orientation.

	speed_controller: NON_LINEAR_SPEED
			-- Controller for the robot linear speed.

	time_handler: TIME_PARSER
			-- ROS message time stamp parser.

	linear_speed: REAL_64
			-- Desiread linear speed.

	angular_velocity: REAL_64
			-- Desired angular speed.
end
