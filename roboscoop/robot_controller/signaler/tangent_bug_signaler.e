note
	description: "State of tangent bug behaviour."
	author: "Ferran Pallarès"
	date: "06.11.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (goal_parameters: separate GOAL_PARAMETERS; pid_parameters: separate PID_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Initialize signaler with attributes.
		do
			create goal.make_with_coordinates (goal_parameters.x, goal_parameters.y)
			point_reached_threshold := goal_parameters.threshold -- TODO - put at the most convenient parser.
			revisited_point_threshold := goal_parameters.threshold -- TODO - put at most convenient parser.
			minimum_distance_to_goal := 0 -- TODO - check - {REAL_64}.positive_infinity
			create obstacle_entry_point.make
		end

feature -- Access

	goal: POINT_2D
			-- Current goal point.

	point_reached_threshold: REAL_64
			-- Distance from robot to point to consider it reached.

	revisited_point_threshold: REAL_64
			-- Minimum distance from a point to begin considering having already visited it.

	minimum_distance_to_goal: REAL_64
			-- Minimum recorded distance from obstacle to goal.

	obstacle_entry_point: POINT_2D
			-- Point where robot first sensed an obstacle.

	has_left_obstacle_entry_point: BOOLEAN
			-- Whether the robot has left the current entry point.

	set_minimum_distance_to_goal (min_distance: REAL_64)
			-- Setter for `minimum_distance_to_goal'.
		do
			minimum_distance_to_goal := min_distance
		end

	set_obstacle_entry_point (point: separate POINT_2D)
			-- Setter for `obstacle_entry_point'.
		local
			l_point: POINT_2D
		do
			create l_point.make_from_separate (point)
			obstacle_entry_point := l_point
		end

	set_has_left_obstacle_entry_point (value: BOOLEAN)
			-- Setter for `has_left_obstacle_entry_point'.
		do
			has_left_obstacle_entry_point := value
		end

feature -- State

	is_go_to_goal_pending: BOOLEAN
			-- Has the state "go_to_goal" been handled by the algorithm?

	is_follow_obstacle_pending: BOOLEAN
			-- Has the state "follow_obstacle" been handled by the algorithm?

	is_leave_obstacle_pending: BOOLEAN
			-- Has the state "leave_obstacle" been handled by the algorithm?

	is_reached_goal_pending: BOOLEAN
			-- Has the state "reached_goal" been handled by the algorithm?

	is_unreachable_goal_pending: BOOLEAN
			-- Has the state "unreachable_goal" been handled by the algorithm?

	set_is_go_to_goal_pending (a_val: BOOLEAN)
			-- Set `is_go_to_goal_pending' value equal to `a_val'.
		do
			is_go_to_goal_pending := a_val
		end

	set_is_follow_obstacle_pending (a_val: BOOLEAN)
			-- Set `is_follow_obstacle_pending' value equal to `a_val'.
		do
			is_follow_obstacle_pending := a_val
		end

	set_is_leave_obstacle_pending (a_val: BOOLEAN)
			-- Set `is_leave_obstacle_pending' value equal to `a_val'.
		do
			is_leave_obstacle_pending := a_val
		end

	set_is_reached_goal_pending (a_val: BOOLEAN)
			-- Set `is_reached_goal_pending' value equal to `a_val'.
		do
			is_reached_goal_pending := a_val
		end

	set_is_unreachable_goal_pending (a_val: BOOLEAN)
			-- Set `is_unreachable_goal_pending' value equal to `a_val'.
		do
			is_unreachable_goal_pending := a_val
		end

	clear_all_pendings
			-- Set all pending flags to False.
		do
			is_go_to_goal_pending := False
			is_follow_obstacle_pending := False
			is_leave_obstacle_pending := False
			is_reached_goal_pending := False
			is_unreachable_goal_pending := False
		end

feature {TANGENT_BUG_BEHAVIOUR, TANGENT_BUG_CONTROLLER} -- Require check

	is_minimum_distance_recorded: BOOLEAN
			-- TODO
		do
			Result := minimum_distance_to_goal > 0
		end

	is_closer_than_minimum_distance (o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Whether the current distance to goal is lower than minimum recorded distance.
		do
			Result := goal.get_euclidean_distance (get_current_position (o_sig)) < minimum_distance_to_goal
		end

	is_obstacle_sensed (r_g: separate RANGE_GROUP): BOOLEAN
			-- Whether an obstacle is sensed in front of the robot.
		do
			Result := r_g.is_obstacle_in_front
		end

	is_goal_reached (o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Whether the goal has been reached.
		do
			Result := goal.get_euclidean_distance (get_current_position (o_sig)) < point_reached_threshold
		end

	is_closer_safe_point_sensed (r_g: separate RANGE_GROUP): BOOLEAN
			-- Whether there is a safe sensed point closer to the goal than minimum recorded distance.
		local
			safe_point: POINT_2D
		do
			create safe_point.make_from_separate (r_g.get_closest_safe_point_in_front (goal))
			Result := safe_point.get_euclidean_distance (goal) < minimum_distance_to_goal -- TODO - cal? -  not (safe_point.get_x = 0 and safe_point.get_y = 0)
        end

	is_goal_unreachable (o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Determines whether the robot has entered the point where it started following an obstacle.
		local
			robot_position: POINT_2D
			distance_robot_to_obstacle_entry_point: REAL_64
		do
			robot_position := create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y)
			distance_robot_to_obstacle_entry_point := robot_position.get_euclidean_distance (obstacle_entry_point)

			if distance_robot_to_obstacle_entry_point >= revisited_point_threshold then
				set_has_left_obstacle_entry_point (True)	-- TODO follow wall and leave wall have to handle this to set it to false each time we deal with another obstacle!
			end

			Result := distance_robot_to_obstacle_entry_point < point_reached_threshold and has_left_obstacle_entry_point
		end

	set_goal_coordinates (x, y: REAL_64)
		do
			create goal.make_with_coordinates (x, y)
		end

feature {NONE} -- Implementation

	get_current_position (o_sig: separate ODOMETRY_SIGNALER): POINT_2D
			-- Return current pose.
		local
			point: POINT_2D
		do
			create point.make_with_coordinates (o_sig.x, o_sig.y)
			Result := point
		end
end
