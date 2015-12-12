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

	make_with_attributes (a_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Create self.
		do
			is_clockwise := False
			safe_leaving_wall_vertical_distance := wall_following_parameters.safe_leaving_wall_vertical_distance
			distance_from_wall := wall_following_parameters.desired_wall_distance
			corner_offset := create {POINT_2D}.make_from_separate (wall_following_parameters.safe_outer_corner_turn_offset)

			create world_tf.make
			create target_point.make
			create last_point.make
			create pose_controller.make_with_attributes (a_pose_controller_parameters.pid_parameters, a_pose_controller_parameters.nlsc_parameters,
															a_pose_controller_parameters.turning_angular_speed, a_pose_controller_parameters.reached_point_threshold,
															a_pose_controller_parameters.reached_orientation_threshold)
		end

feature -- Access

	update_velocity (drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			pose_controller.update_drive_velocity (drive)
		end

	set_readings (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- <Precursor>
		do
			pose_controller.set_current_pose (t_sig.current_pose, t_sig.timestamp)

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
				t_sig.set_go_to_goal
			end

		end

	set_clockwise
			-- Set clockwise wall-following.
		do
			corner_offset.make_with_coordinates (corner_offset.get_x, {DOUBLE_MATH}.dabs (corner_offset.get_y))
			is_clockwise := True
		end

	set_counter_clockwise
			-- Set counter-clockwise wall-following.
		do
			corner_offset.make_with_coordinates (corner_offset.get_x, -{DOUBLE_MATH}.dabs (corner_offset.get_y))
			is_clockwise := False
		end

feature {NONE} -- Implementation

	safe_leaving_wall_vertical_distance: REAL_64
			-- Minimum local vertical distance between the robot and the followed wall to leave it safely.

	distance_from_wall: REAL_64
			-- Distance to follow wall.

	corner_offset: POINT_2D
			-- Target offset when finding a corner.

	world_tf: TRANSFORM_2D
			-- Transformation from base to robot.

	target_point: POINT_2D
			-- Target point, offset from last detected point.

	last_point: POINT_2D
			-- Last point detected in the wall the robot is following.

	pose_controller: POSE_CONTROLLER
			-- Pose controller.

	is_clockwise: BOOLEAN
			-- If the wall is being followed in clockwise direction.

	follow_parallel_wall (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow parallel wall.
		local
			heading: REAL_64
		do
			world_tf.make_with_offsets (t_sig.current_pose.get_position.get_x, t_sig.current_pose.get_position.get_y, t_sig.current_pose.get_orientation)
			target_point := world_tf.project_to_parent (corner_offset)
			last_point := world_tf.project_to_parent (create {POINT_2D}.make_from_separate (range_signaler.get_closest_obstacle_point))

			if is_clockwise then
				heading := range_signaler.follow_right_wall (distance_from_wall)
			else
				heading := range_signaler.follow_left_wall (distance_from_wall)
			end

			pose_controller.set_target_pose (create {POSE_2D}.make_with_pose (get_point_in_direction (t_sig.current_pose, pose_controller.reached_point_threshold*2.0, heading),
												t_sig.current_pose.get_orientation + heading))

			debug
				io.put_string ("Follow Wall %N")
			end
		end

	follow_inner_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow wall inner corner.
		local
			heading: REAL_64
		do
			if is_clockwise then
				heading := range_signaler.follow_closest_wall_cw (distance_from_wall)
			else
				heading := range_signaler.follow_closest_wall_ccw (distance_from_wall)
			end

			pose_controller.set_target_pose (create {POSE_2D}.make_with_pose (get_point_in_direction (t_sig.current_pose, pose_controller.reached_point_threshold*2.0, heading),
												t_sig.current_pose.get_orientation + heading))

			debug
				io.put_string ("Inner Corner %N")
			end
		end

	follow_outer_corner (t_sig: separate TANGENT_BUG_SIGNALER; range_signaler: separate RANGE_GROUP)
			-- Proceed to follow wall outer corner.
		local
			heading: REAL_64
			math: TRIGONOMETRY_MATH
		do
			create math
			heading := math.atan2 (target_point.get_y - t_sig.current_pose.get_position.get_y, target_point.get_x - t_sig.current_pose.get_position.get_x) - t_sig.current_pose.get_orientation
			heading := math.atan2 (math.sine (heading), math.cosine (heading))
			pose_controller.set_target_pose (create {POSE_2D}.make_with_pose (target_point, t_sig.current_pose.get_orientation + heading))
			if not pose_controller.is_target_position_reached then
				pose_controller.set_current_pose (t_sig.current_pose, t_sig.timestamp)
			else
				target_point := last_point
			end

			debug
				io.put_string ("Outer Corner %N")
			end
		end

	update_minimum_distance_to_goal (t_sig: separate TANGENT_BUG_SIGNALER)
			-- Check if current distance to goal is smaller than the minimum
		do
			if t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) < t_sig.min_distance then
				t_sig.set_min_distance (t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position))
			end
		end

	get_point_in_direction (pose: separate POSE_2D; distance, heading: REAL_64): POINT_2D
			-- Compute a point at the given distance from the given position in the given relative direction.
		local
			x_increment, y_increment: REAL_64
		do
			x_increment := distance*{DOUBLE_MATH}.cosine (pose.get_orientation + heading)
			y_increment := distance*{DOUBLE_MATH}.sine (pose.get_orientation + heading)
			Result := create {POINT_2D}.make_with_coordinates (pose.get_position.get_x + x_increment, pose.get_position.get_y + y_increment)
		end
end
