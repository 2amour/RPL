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

	make(topic_name: separate STRING; range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS)
			-- Create an array of sensors and register them.
		do
			register_transforms (range_sensors_parameters)
			make_with_topic (topic_name)
			register_ranges
			close_obstacle_threshold := range_sensors_parameters.close_obstacle_threshold
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

	register_transforms (range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS)
			-- Register Thymio sensor offsets
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

feature -- Access

	is_obstacle: BOOLEAN
			-- <Precursor>
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
			-- <Precursor>
		do
			Result := 	sensors[1].is_valid_range or sensors[2].is_valid_range or sensors[3].is_valid_range or
						sensors[4].is_valid_range or sensors[5].is_valid_range
		end

	is_front_obstacle_close: BOOLEAN
			-- <Precursor>
		do
			Result := {DOUBLE_MATH}.dabs (sensors[1].range) < close_obstacle_threshold or
						{DOUBLE_MATH}.dabs (sensors[2].range) < close_obstacle_threshold or
						{DOUBLE_MATH}.dabs (sensors[3].range) < close_obstacle_threshold or
						{DOUBLE_MATH}.dabs (sensors[4].range) < close_obstacle_threshold or
						{DOUBLE_MATH}.dabs (sensors[5].range) < close_obstacle_threshold
		end

	is_obstacle_at_back: BOOLEAN
			-- <Precursor>
		do
			Result := sensors[6].is_valid_range or sensors[7].is_valid_range
		end

	is_obstacle_mostly_at_left: BOOLEAN
			-- <Precursor>
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
			-- <Precursor>
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
			-- <Precursor>
		do
			Result := 	(sensors[1].is_valid_range and sensors[2].is_valid_range and sensors[3].is_valid_range and
						sensors[4].is_valid_range and sensors[5].is_valid_range) or
						(sensors[6].is_valid_range and sensors[7].is_valid_range)
		end

	is_all_front_sensors_open: BOOLEAN
			-- <Precursor>
		do
			Result := not is_obstacle_in_front
		end

	is_wall_only_at_left: BOOLEAN
			-- <Precursor>
		do
			Result := sensors[1].is_valid_range and
					  (not sensors[3].is_valid_range) and
					  (not sensors[4].is_valid_range) and
					  (not sensors[5].is_valid_range)
		end

	is_wall_only_at_right: BOOLEAN
			-- <Precursor>
		do
			Result := sensors[5].is_valid_range and
					  (not sensors[3].is_valid_range) and
					  (not sensors[2].is_valid_range) and
					  (not sensors[1].is_valid_range)
		end

	is_huge_at_left: BOOLEAN
			-- <Precursor>
		do
			Result := (sensors[3].is_valid_range or sensors[4].is_valid_range or sensors[5].is_valid_range)
		end

	is_huge_at_right: BOOLEAN
			-- <Precursor>
		do
			Result := (sensors[3].is_valid_range or sensors[2].is_valid_range or sensors[1].is_valid_range)
		end

	is_sensor_at_front (sensor_index: INTEGER_32): BOOLEAN
			-- <Precursor>
		do
			Result := (sensor_index > 1 and sensor_index < 5)
		end

	get_left_wall: LINE_2D
			-- <Precursor>
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
			-- <Precursor>
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
			-- <Precursor>
		local
			p1, p2: POINT_2D
		do
			p1 := transforms[i].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[i].range, 0))
			if i /= 3 then
				create p2.make_with_coordinates (p1.get_x + 1, p1.get_y)
			else
				create p2.make_with_coordinates (p1.get_x, p1.get_y + 1)
			end
			Result := create {LINE_2D}.make_with_points (p1, p2)
		end

	get_line_between_sensors (i, j: INTEGER_32): LINE_2D
			-- <Precursor>
		local
			p1, p2: POINT_2D
		do
			p1 := transforms[i].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[i].range, 0))
			p2 := transforms[j].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[j].range, 0))

			Result := create {LINE_2D}.make_with_points (p1, p2)
		end

	get_closest_wall_from_point (p: POINT_2D): LINE_2D
			-- <Precursor>
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

			from i := 1 until i > 7
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

	follow_left_wall (desired_distance: REAL_64): REAL_64
			-- <Precursor>
		do
			Result := follow_line_ccw (desired_distance, get_left_wall)
		end

	follow_right_wall (desired_distance: REAL_64): REAL_64
			-- <Precursor>
		do
			Result := follow_line_cw (desired_distance, get_right_wall)
		end

	follow_closest_wall_ccw (desired_distance: REAL_64): REAL_64
			-- <Precursor>
		do
			Result := follow_line_ccw (desired_distance, get_closest_wall_from_point (create {POINT_2D}.make_with_coordinates (0, 0)))
		end

	follow_closest_wall_cw (desired_distance: REAL_64): REAL_64
			-- <Precursor>
		do
			Result := follow_line_cw (desired_distance, get_closest_wall_from_point (create {POINT_2D}.make_with_coordinates (0, 0)))
		end

	follow_line_ccw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- <Precursor>
		local
			current_distance: REAL_64
			v_wall: VECTOR_2D
			v_robot: VECTOR_2D
			v_theta: VECTOR_2D
		do
			current_distance := line.get_distance_from_point (create {POINT_2D}.make_with_coordinates (0.0, 0.0))
			v_wall := line.get_vector.get_unitary
			v_robot := v_wall.get_perpendicular

			v_theta := (v_wall*desired_distance) + (v_robot*((current_distance - desired_distance)))
			Result := v_theta.get_angle
		end

	follow_line_cw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- <Precursor>
		local
			current_distance: REAL_64
			v_wall: VECTOR_2D
			v_robot: VECTOR_2D
			v_theta: VECTOR_2D
		do
			current_distance := line.get_distance_from_point (create {POINT_2D}.make_with_coordinates (0.0, 0.0))
			v_wall := line.get_vector.get_unitary
			v_robot := v_wall.get_perpendicular

			v_theta := (v_wall*desired_distance) - (v_robot*((current_distance - desired_distance)))
			Result := v_theta.get_angle
		end

	follow_line (desired_distance: REAL_64; line: LINE_2D; clockwise: BOOLEAN): REAL_64
			-- <Precursor>
		do
		end

	get_sensor_point (sensor_index: INTEGER_32): POINT_2D
			-- <Precursor>
		do
			if sensors[sensor_index].is_valid_range then
				Result := transforms[sensor_index].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[sensor_index].range, 0.0))
			else
				Result := transforms[sensor_index].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[sensor_index].max_range, 0.0))
			end
		end

	get_perpendicular_minimum_distance_to_wall: REAL_64
			-- <Precursor>
		local
			left_distance, right_distance: REAL_64
		do
			left_distance := {DOUBLE_MATH}.dabs (get_sensor_point (1).get_y)
			right_distance := {DOUBLE_MATH}.dabs (get_sensor_point (5).get_y)
			if left_distance < right_distance then
				Result := left_distance
			else
				Result := right_distance
			end
		end

	get_closest_obstacle_point: POINT_2D
			-- Get closest sensed obstacle point.
		local
			i: INTEGER
			closest_point: POINT_2D
			minimum_range: REAL_64
		do
			create closest_point.make
			minimum_range := {REAL_64}.max_value
			from i := sensors.lower until i > sensors.upper
			loop
				if sensors[i].is_valid_range and sensors[i].range < minimum_range then
					minimum_range := sensors[i].range
					closest_point := transforms[i].project_to_parent (get_sensor_point (i))
				end
				i := i + 1
			end
			Result := closest_point
		end

feature {NONE} -- Implementation

	close_obstacle_threshold: REAL_64
			-- Distance for considering an obstacle to be close.
end
