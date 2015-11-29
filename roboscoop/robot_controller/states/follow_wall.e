note
	description: "In this state the robot follows the perimeter of an obstacle."
	author: "Ferran Pallarès"
	date: "16.10.2015"

class
	FOLLOW_WALL

inherit
	TANGENT_BUG_STATE

create
	make_with_attributes

feature -- Initialization

	make_with_attributes (pid_parameters: separate PID_PARAMETERS; nlsc_parameters: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Create self.
		do
			is_clockwise := False
			safe_leaving_wall_vertical_distance := wall_following_parameters.safe_leaving_wall_vertical_distance
			turning_angular_velocity := wall_following_parameters.outer_corner_angular_velocity
			target_threshold := wall_following_parameters.safe_outer_corner_turn_offset_threshold
			distance_from_wall := wall_following_parameters.desired_wall_distance
			corner_offset := create {POINT_2D}.make_from_separate (wall_following_parameters.safe_outer_corner_turn_offset)

			create world_tf.make
			create target_point.make
			create orientation_controller.make_with_gains (pid_parameters.kp, pid_parameters.ki, pid_parameters.kd)
			create speed_controller.make_with_attributes (nlsc_parameters.maximum_speed, nlsc_parameters.angular_decay_rate)
			create time_handler.start (0.0)
			create last_point.make
		end

feature -- Access

	update_velocity (drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			drive.set_velocity (linear_speed, angular_velocity)
		end

	set_readings (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- <Precursor>
		do

			time_handler.set_time (t_sig.timestamp)
			orientation_controller.set_sampling (time_handler.get_sampling_rate)

			if time_handler.get_sampling_rate > 0 then
				if is_clockwise then
					if range_signaler.is_wall_only_at_right then
						follow_parallel_wall (t_sig, range_signaler)
					elseif range_signaler.is_huge_at_left then
						follow_inner_corner (t_sig, range_signaler)
					elseif range_signaler.is_all_front_sensors_open then
						follow_outer_corner (t_sig, range_signaler)
					end
				else
					if range_signaler.is_wall_only_at_left then
						follow_parallel_wall (t_sig, range_signaler)
					elseif range_signaler.is_huge_at_right then
						follow_inner_corner (t_sig, range_signaler)
					elseif range_signaler.is_all_front_sensors_open then
						follow_outer_corner (t_sig, range_signaler)
					end
				end

				update_minimum_distance_to_goal (t_sig)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		local
			v_leave_point, best_point: POINT_2D
			vector_to_goal: VECTOR_2D
			world_coordinates: TRANSFORM_2D
			i : INTEGER_32

			min_distance: REAL_64
		do
			create world_coordinates.make_with_offsets (t_sig.current_pose.get_position.get_x, t_sig.current_pose.get_position.get_y, t_sig.current_pose.get_orientation)
			create best_point.make
			create v_leave_point.make
			min_distance :=  {REAL_64}.max_value

			from i := r_sig.sensors.lower
			until i >= r_sig.sensors.upper
			loop
				if not r_sig.is_obstacle_in_front and
					r_sig.is_sensor_at_front (i) and
					r_sig.get_perpendicular_minimum_distance_to_wall > safe_leaving_wall_vertical_distance then
						v_leave_point := world_coordinates.project_to_parent (create {POINT_2D}.make_with_coordinates (r_sig.get_sensor_point (i).get_x, r_sig.get_sensor_point (i).get_y))
				end

				if v_leave_point.get_euclidean_distance (t_sig.goal) < min_distance then
					min_distance := v_leave_point.get_euclidean_distance (t_sig.goal)
					best_point := v_leave_point
				end
				i := i + 1
			end

			if min_distance < t_sig.min_distance then
				t_sig.set_leave_wall_with_target (best_point)
			end

			if t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.reached_point_threshold then
				t_sig.set_at_goal
			end

		end

	set_clockwise
			-- Set clockwise wall-following.
		do
			corner_offset.make_with_coordinates (corner_offset.get_x, {DOUBLE_MATH}.dabs (corner_offset.get_y))
			turning_angular_velocity := -{DOUBLE_MATH}.dabs (turning_angular_velocity)
			is_clockwise := True
		end

	set_counter_clockwise
			-- Set counter-clockwise wall-following.
		do
			corner_offset.make_with_coordinates (corner_offset.get_x, -{DOUBLE_MATH}.dabs (corner_offset.get_y))
			turning_angular_velocity := {DOUBLE_MATH}.dabs (turning_angular_velocity)
			is_clockwise := False
		end

feature {NONE} -- Implementation

	safe_leaving_wall_vertical_distance: REAL_64
			-- Minimum local vertical distance between the robot and the followed wall to leave it safely.

	turning_angular_velocity: REAL_64
			-- Angular velocity when its only turning.

	distance_from_wall: REAL_64
			-- Distance to follow wall.

	corner_offset: POINT_2D
			-- Target offset when finding a corner.

	world_tf: TRANSFORM_2D
			-- Transformation from base to robot.

	target_point: POINT_2D
			-- Target point, offset from last detected point.

	target_threshold: REAL_64
			-- Target threshold for controller.

	last_point: POINT_2D
			-- Last point detected in the wall the robot is following.

	orientation_controller: PID_CONTROLLER
			-- Controller for the robot orientation.

	speed_controller: NON_LINEAR_SPEED_CONTROLLER
			-- Controller for the robot linear speed.

	time_handler: TIME_HANDLER
			-- Time stamp manager.

	linear_speed: REAL_64
			-- Desiread linear speed.

	angular_velocity: REAL_64
			-- Desired angular speed.

	is_clockwise: BOOLEAN
			-- If the wall is being followed in clockwise direction.

	follow_parallel_wall (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow parallel wall.
		local
			error: REAL_64
		do
			world_tf.make_with_offsets (t_sig.current_pose.get_position.get_x, t_sig.current_pose.get_position.get_y, t_sig.current_pose.get_orientation)
			target_point := world_tf.project_to_parent (corner_offset)

			if is_clockwise then
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

	follow_inner_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow wall inner corner.
		local
			error: REAL_64
		do
			if is_clockwise then
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

	follow_outer_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow wall outer corner.
		local
			error: REAL_64
			line: LINE_2D
			math: TRIGONOMETRY_MATH
		do
			create math
			if target_point.get_euclidean_distance (t_sig.current_pose.get_position) > target_threshold then
				error := math.atan2 (target_point.get_y - t_sig.current_pose.get_position.get_y, target_point.get_x - t_sig.current_pose.get_position.get_x)  - t_sig.current_pose.get_orientation
				error := math.atan2 (math.sine (error), math.cosine (error))
				orientation_controller.set_error (error)
				angular_velocity := orientation_controller.get_output
			else
				angular_velocity := turning_angular_velocity
				speed_controller.set_angular_velocity (angular_velocity)
				linear_speed := speed_controller.get_output
			end

		end

	update_minimum_distance_to_goal (t_sig: separate TANGENT_BUG_SIGNALER)
			-- Check if current distance to goal is smaller than the minimum
		do
			if t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.min_distance then
				t_sig.set_min_distance (t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position))
			end
		end
end
