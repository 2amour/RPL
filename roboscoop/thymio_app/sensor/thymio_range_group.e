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
	PERIMETER_OBSTACLE_DETECTOR

create
	make

feature {NONE} -- Initialization.

	make(topic_name: separate STRING)
			-- Create an array of sensors and register them.
		do
			register_transforms
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

	register_transforms
			-- Register Thymio sensor offsets
		local
			data_parser: RANGE_SENSOR_PARSER
		do
			create data_parser.make_with_path ("sensor_coordinates.txt")
			create transforms.make_from_array (data_parser.transforms)
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
				Result := Result and (sensor.item.range > ({THYMIO_ROBOT}.robot_base_size - 8.0))
			end
		end

	is_all_front_sensors_open: BOOLEAN
			-- <Precursor>
		do
			-- TODO.
			Result := False
		end

	open_direction_front: VECTOR_3D_MSG
			-- <Precursor>
		do
			-- TODO.
			Result := create {VECTOR_3D_MSG}.make_empty
		end

	follow_wall_orientation (a_desired_distance_from_wall: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the wall
		local
			a: POINT_2D
			b: POINT_2D
			--v_wall:  LINE_2D
			v_wall: VECTOR_2D
			v_robot: VECTOR_2D
			current_distance: REAL_64
			v_theta: VECTOR_2D
		do
			if is_obstacle_mostly_at_left then
				a := transforms[1].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[1].range, 0))
				if sensors[2].is_valid_range then
					b := transforms[2].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[2].range, 0))
				else
					create b.make_with_coordinates (a.get_x + 0.1, a.get_y)
				end

			else
				a := transforms[5].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[5].range, 0))

				if sensors[4].is_valid_range then
					b := transforms[4].project_to_parent (create {POINT_2D}.make_with_coordinates (sensors[4].range, 0))
				else
					create b.make_with_coordinates (a.get_x + 0.1, a.get_y)
				end

			end


			v_wall := (create {LINE_2D}.make_with_points (a, b)).get_vector.get_unitary

			if is_obstacle_mostly_at_left then
				v_robot := v_wall.get_perpendicular.get_scaled (-1)
			else
				v_robot := v_wall.get_perpendicular
			end

			current_distance :=  {DOUBLE_MATH}.dabs (v_robot.dot (create {VECTOR_2D}.make_with_coordinates (b.get_x, b.get_y)))

			v_theta := v_wall.get_scaled (a_desired_distance_from_wall).add (v_robot.get_scaled (current_distance - a_desired_distance_from_wall))

			Result := v_theta.get_angle

		end

end
