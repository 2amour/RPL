note
	description: 	"[
						Group of Thymio's horizontal range sensors.
						Detects obstacles around the robot.
					]"
	author: "Rusakov Andrey"
	date: "18.09.2013"

class
	THYMIO_RANGE_GROUP

inherit
	GROUP_SIGNALER[RANGE_MSG]
	RANGE_GROUP

create
	make

feature {NONE} -- Initialization.

	make (topic_name: separate STRING; range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS)
			-- Create an array of sensors and register them.
		do
			initialize_sensors_transforms (range_sensors_parameters)
			make_with_topic (topic_name)
			register_ranges
		end

	register_ranges
			-- Register Thymio ground sensors by their frame ids.
		do
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_0)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_1)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_2)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_3)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_4)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_5)
			register_sensor ({THYMIO_TOPICS}.prox_horizontal_link_6)
		end

	initialize_sensors_transforms (range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS)
			-- Initialize transforms with sensors position offsets and orientation.
		local
			i: INTEGER
			pose: POSE_2D
		do
			create transforms.make_filled (create {TRANSFORM_2D}.make, range_sensors_parameters.sensors_poses.lower, range_sensors_parameters.sensors_poses.upper)
			from i := range_sensors_parameters.sensors_poses.lower
			until i > range_sensors_parameters.sensors_poses.upper
			loop
				create pose.make_from_separate (range_sensors_parameters.sensors_poses[i])
				transforms.put (create {TRANSFORM_2D}.make_with_offsets (pose.get_position.get_x, pose.get_position.get_y, pose.get_orientation), i)
				i := i + 1
			end
		end

feature -- Access.
	transforms: ARRAY[TRANSFORM_2D]

	is_obstacle: BOOLEAN
			-- Whether an obstacle is observed by any sensor in valid range?
		local
			i: INTEGER
		do
			from
				i := sensors.lower
			until
				i > sensors.upper or Result
			loop
				Result := Result or sensors[i].is_valid_range
				i := i + 1
			end
		end

	is_obstacle_in_front: BOOLEAN
			-- Whether an obstacle is observed in front?
		do
			Result := 	sensors[1].is_valid_range or sensors[2].is_valid_range or sensors[3].is_valid_range or
						sensors[4].is_valid_range or sensors[5].is_valid_range
		end

	is_obstacle_at_back: BOOLEAN
			-- Whether an obstacle is observed at back?
		do
			Result := sensors[6].is_valid_range or sensors[7].is_valid_range
		end

	is_obstacle_mostly_at_left: BOOLEAN
			-- Whether an obstacle is observed at left?
		local
			i: INTEGER
			left_sum, right_sum: REAL_32
		do
			from
				i := sensors.lower
			until
				i > sensors.upper
			loop
				if sensors[i].is_valid_range then
					if i = 1 or i = 2 then --or i = 6 then
						left_sum := left_sum + sensors[i].range
					end
					if i = 4 or i = 5 then --or i = 7 then
						right_sum := right_sum + sensors[i].range
					end
				end
				i := i + 1
			end

			Result := (left_sum > 0 and left_sum < right_sum) or (left_sum > 0 and right_sum <= 0)
		end

	is_obstacle_mostly_at_right: BOOLEAN
			-- Whether an obstacle is observed at right?
		local
			i: INTEGER
			left_sum, right_sum: REAL_32
		do
			from
				i := sensors.lower
			until
				i > sensors.upper
			loop
				if sensors[i].is_valid_range then
					if i = 1 or i = 2 then --or i = 6 then
						left_sum := left_sum + sensors[i].range
					end
					if i = 4 or i = 5 then --or i = 7 then
						right_sum := right_sum + sensors[i].range
					end
				end
				i := i + 1
			end

			Result := (right_sum > 0 and left_sum > right_sum) or (right_sum > 0 and left_sum <= 0)
		end

	is_obstacle_huge: BOOLEAN
			-- Is obstacle observed by majority of the sensors?
		do
			Result := 	(sensors[1].is_valid_range and sensors[2].is_valid_range and sensors[3].is_valid_range and
						sensors[4].is_valid_range and sensors[5].is_valid_range) or
						(sensors[6].is_valid_range and sensors[7].is_valid_range)
		end

	is_front_sensor (a_index: INTEGER): BOOLEAN
			-- <Precursor>
		do
			Result := a_index = 3
		end

	hit_point_front (a_sensor_index: INTEGER): VECTOR_3D_MSG
			-- <Precursor>
		local
			point: POINT_2D
			range: REAL_64
		do
			range := sensors[a_sensor_index].range
			point := transforms[a_sensor_index].project_to_parent ( create {POINT_2D}.make_with_coordinates (range, 0.0) )
			Result := point.get_vector_3d_msg
		end

	has_obstacle (a_direction_with_respect_to_robot: REAL_64): BOOLEAN
			-- <Precursor>
			-- If direction is pi/8 rads away of a sensor with a valid_range measurement then has obstacle
		local
			i: INTEGER_32
		do
			i := 1
			Result := False
			across sensors as sensor
			loop
				Result := Result or
					(sensor.item.is_valid_range and {DOUBLE_MATH}.dabs (a_direction_with_respect_to_robot - transforms[i].get_heading) <= {TRIGONOMETRY_MATH}.pi_8)
				i := i + 1
			end
		end

	is_enough_space_for_changing_direction: BOOLEAN
			-- <Precursor>
		do
			Result := True
			across sensors as sensor
			loop
				Result := Result and (sensor.item.range > ({THYMIO_ROBOT}.robot_base_size - 8.0)) -- TODO, make it with transforms
			end
		end

	is_all_front_sensors_open: BOOLEAN
			-- <Precursor>
		do
			Result := not is_obstacle_in_front
		end

	open_direction_front: VECTOR_3D_MSG
			-- <Precursor>
		do
			-- TODO.
			Result := create {VECTOR_3D_MSG}.make_empty
		end

	is_wall_only_at_left: BOOLEAN
		-- Check if there is a wall only at left.
		do
			Result := sensors[1].is_valid_range and
					  (not sensors[3].is_valid_range) and
					  (not sensors[4].is_valid_range) and
					  (not sensors[5].is_valid_range)
		end

	is_wall_only_at_right: BOOLEAN
		-- Check if there is a wall only at right.
		do
			Result := sensors[5].is_valid_range and
					  (not sensors[3].is_valid_range) and
					  (not sensors[2].is_valid_range) and
					  (not sensors[1].is_valid_range)
		end

	is_huge_at_left: BOOLEAN
		-- Check if there is an obstacle in left and right sensors
		do
			Result := (sensors[3].is_valid_range or sensors[4].is_valid_range or sensors[5].is_valid_range)
		end

	is_huge_at_right: BOOLEAN
		-- Check if there is an obstacle in left and right sensors
		do
			Result := (sensors[3].is_valid_range or sensors[2].is_valid_range or sensors[1].is_valid_range)
		end

	get_left_wall: LINE_2D
			-- Get left wall.
		local
			line: LINE_2D
		do
			if sensors[2].is_valid_range then
				Result := get_line_between_sensors (1, 2)
			else
				Result := get_estimated_line_from_sensor (1)
			end

		end

	get_right_wall: LINE_2D
			-- Get right wall.
		local
			line: LINE_2D
		do
			if sensors[4].is_valid_range then
				Result := get_line_between_sensors (4, 5)
			else
				Result := get_estimated_line_from_sensor (5)
			end

		end

	get_estimated_line_from_sensor (i: INTEGER_32): LINE_2D
			-- Get estimated line from sensor readings. When sensors are in valid range then its the line joining the points, else its an estimation.
--		require -- TODO - revise
--			valid_range: (i >= 1 and i <= 7)
		local
			p1, p2: POINT_2D
		do
			p1 := transforms[i].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[i].range, 0))
			if not is_front_sensor (i) then
				create p2.make_with_coordinates (p1.get_x + 1, p1.get_y)
			else
				create p2.make_with_coordinates (p1.get_x, p1.get_y + 1)
			end
			Result := create {LINE_2D}.make_with_points (p1, p2)
		end

	get_line_between_sensors (i, j: INTEGER_32): LINE_2D
			-- Get Line between two sensors.
		local
			p1, p2: POINT_2D
		do
			p1 := transforms[i].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[i].range, 0))
			p2 := transforms[j].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[j].range, 0))

			Result := create {LINE_2D}.make_with_points (p1, p2)
		end

	get_closest_wall_from_point (p: POINT_2D): LINE_2D
			-- Find the closest wall from the measured points.
		local
			line, best_line: LINE_2D
			p1, p2: POINT_2D
			distance, minimum_distance: REAL_64
			i, j: INTEGER_32
		do
			create best_line.make
			minimum_distance := {REAL_64}.max_value
			i := 1
			j := 1

			from i:=1 until i > 7
  			loop
  				if sensors[i].is_valid_range then
					from j := 1 until j > 7
					loop
						if sensors[j].is_valid_range then
							line := get_line_between_sensors (i, j)
						else
							line := get_estimated_line_from_sensor (i)
						end

						distance := line.get_distance_from_point (p)
						if (distance < minimum_distance) then
							minimum_distance := distance
							best_line := line
						end

						j := j + 1
					end
  				end
  				i := i + 1
  			end

			Result := best_line
		end

	follow_wall_orientation (a_desired_distance_from_wall: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the wall.
	do
		Result := 0.0
	end

	follow_left_wall (desired_distance: REAL_64): REAL_64
		-- Get the orientation the robot should head in order to follow the left wall.
		do
			Result := follow_line_ccw (desired_distance, get_left_wall)
		end

	follow_right_wall (desired_distance: REAL_64): REAL_64
		-- Get the orientation the robot should head in order to follow the right wall.
		do
			Result := follow_line_cw (desired_distance, get_right_wall)
		end

	follow_closest_wall_ccw (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the closest wall counter-clockwise.
		do
			Result := follow_line_ccw (desired_distance, get_closest_wall_from_point (create {POINT_2D}.make_with_coordinates (0, 0)))
		end

	follow_closest_wall_cw (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the closest wall clockwise.
		do
			Result := follow_line_cw (desired_distance, get_closest_wall_from_point (create {POINT_2D}.make_with_coordinates (0, 0)))
		end

	follow_line_ccw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction when turning counter-clockwise.
		do
			Result := follow_line (desired_distance, line, False)
		end

	follow_line_cw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction when turning clockwise
		do
			Result := follow_line (desired_distance, line, True)
		end

	follow_line (desired_distance: REAL_64; line: LINE_2D; clockwise: BOOLEAN): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction.
		local
			current_distance: REAL_64
			v_wall: VECTOR_2D
			v_robot: VECTOR_2D
			v_theta: VECTOR_2D
		do
			current_distance := line.get_distance_from_point (create {POINT_2D}.make_with_coordinates (0.0, 0.0))
			v_wall := line.get_vector.get_unitary

			if clockwise then
				v_robot := v_wall.get_perpendicular
			else
				v_robot := v_wall.get_perpendicular * (-1)
			end

			v_theta := (v_wall*desired_distance) + (v_robot*((current_distance - desired_distance)))
			Result := v_theta.get_angle
		end

	get_sensor_point (sensor_index: INTEGER_32): POINT_2D
			-- Get point measured by sensor in local coordinates.
		do
			if sensors[sensor_index].is_valid_range then
				Result := transforms[sensor_index].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[sensor_index].range, 0.0))
			else
				Result := transforms[sensor_index].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[sensor_index].max_range, 0.0))
			end
		end


end
