note
	description: "Interface for data gathered by a group of distance sensors."
	author: "Ferran Pallarès"
	date: "05.11.2015"

deferred class
	RANGE_GROUP

inherit

	OBSTACLE_DETECTOR

feature -- Access

	transforms: ARRAY[TRANSFORM_2D]
			-- Array of transforms from sensor coordinate frame to robot coordinate frame.

	sensors: ARRAYED_LIST [RANGE_MSG]
			-- Array of sensors.
		deferred
		end

	is_obstacle: BOOLEAN
			-- Whether an obstacle is observed by any sensor in valid range.
		deferred
		end

	is_obstacle_in_front: BOOLEAN
			-- Whether an obstacle is observed in front.
		deferred
		end

	is_front_obstacle_close: BOOLEAN
			-- Whether the front obstacle is close to the robot.
		deferred
		end

	is_obstacle_at_back: BOOLEAN
			-- Whether an obstacle is observed at back.
		deferred
		end

	is_obstacle_mostly_at_left: BOOLEAN
			-- Whether an obstacle is observed at left.
		deferred
		end

	is_obstacle_mostly_at_right: BOOLEAN
			-- Whether an obstacle is observed at right.
		deferred
		end

	is_obstacle_huge: BOOLEAN
			-- Whether there is an obstacle observed by majority of the sensors.
		deferred
		end

	is_all_front_sensors_open: BOOLEAN
			-- Whether all front sensors are not blocked.
		deferred
		end

	is_wall_only_at_left: BOOLEAN
			-- Check if there is a wall only at left.
		deferred
		end

	is_wall_only_at_right: BOOLEAN
			-- Check if there is a wall only at right.
		deferred
		end

	is_huge_at_left: BOOLEAN
			-- Check if there is an obstacle in left and right sensors
		deferred
		end

	is_huge_at_right: BOOLEAN
			-- Check if there is an obstacle in left and right sensors
		deferred
		end

	is_sensor_at_front (sensor_index: INTEGER_32): BOOLEAN
			-- Whether the sensor is at front of the robot.
		deferred
		end

	get_left_wall: LINE_2D
			-- Get left wall.
		deferred
		end

	get_right_wall: LINE_2D
			-- Get right wall.
		deferred
		end

	get_estimated_line_from_sensor (i: INTEGER_32): LINE_2D
			-- Get estimated line from sensor readings. When sensors are in valid range then its the line joining the points, else its an estimation.
		deferred
		end

	get_line_between_sensors (i, j: INTEGER_32): LINE_2D
			-- Get Line between two sensors.
		require
			valid_range: (i > sensors.lower and i < sensors.upper) and (j > sensors.lower and j < sensors.upper) and (i /= j)
		deferred
		end

	get_closest_wall_from_point (p: POINT_2D): LINE_2D
			-- Find the closest wall from the measured points.
		deferred
		end

	follow_left_wall (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the left wall.
		deferred
		end

	follow_right_wall (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the right wall.
		deferred
		end

	follow_closest_wall_ccw (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the closest wall counter-clockwise.
		deferred
		end

	follow_closest_wall_cw (desired_distance: REAL_64): REAL_64
			-- Get the orientation the robot should head in order to follow the closest wall clockwise.
		deferred
		end

	follow_line_ccw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction when turning counter-clockwise.
		deferred
		end

	follow_line_cw (desired_distance: REAL_64; line: LINE_2D): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction when turning clockwise
		deferred
		end

	follow_line (desired_distance: REAL_64; line: LINE_2D; clockwise: BOOLEAN): REAL_64
			-- Get the orientation the robot should head in order to reach the desired distance from the line in the line's direction.
		deferred
		end

	get_sensor_point (sensor_index: INTEGER_32): POINT_2D
			-- Get point measured by sensor in local coordinates.
		deferred
		end

	get_perpendicular_minimum_distance_to_wall: REAL_64
			-- Get minimum perpendicular distance from robot to wall.
		deferred
		end

	get_closest_obstacle_point: POINT_2D
			-- Get closest sensed obstacle point.
		deferred
		end
end
