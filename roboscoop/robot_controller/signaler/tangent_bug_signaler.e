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

	set_obstacle_entry_point (point: POINT_2D)
			-- Setter for `obstacle_entry_point'.
		do
			obstacle_entry_point := point
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

end
