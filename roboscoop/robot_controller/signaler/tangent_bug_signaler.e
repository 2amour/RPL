note
	description: "State of Tangent bug."
	author: "Sebastian Curi"
	date: "18.10.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (goal_parameters: separate GOAL_PARAMETERS; pid_parameters: separate PID_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Initialize signaler with attributes.
		do
			create current_pose.make
			create goal.make_with_coordinates (0.0, 0.0)
			goal_threshold := goal_parameters.threshold
			initialize_states (create {PID_PARAMETERS}.make_from_separate (pid_parameters), create {WALL_FOLLOWING_PARAMETERS}.make_from_separate (wall_following_parameters))
			set_go_to_goal
			create intial_point_wall.make
			d_min := {REAL_64}.max_value
		end

	initialize_states (pid_parameters: PID_PARAMETERS; wall_following_parameters: WALL_FOLLOWING_PARAMETERS)
			-- Initialize states.
		do
			create at_goal
			create follow_wall.make_with_attributes (pid_parameters, wall_following_parameters)
			create go_to_goal.make_with_attributes (pid_parameters)
			create leave_wall.make_with_attributes (pid_parameters)
			create unreachable_goal
		end

feature -- Access

	get_goal: POINT_2D
			-- Get goal coordinates.
		do
			Result := goal
		end

	get_goal_threshold: REAL_64
			-- Get threshold for goal.
		do
			Result := goal_threshold
		end

	get_d_min: REAL_64
			-- Get minimum distance to goal.
		do
			Result := d_min
		end

	get_state: TANGENT_BUG_STATE
			-- Get current state.
		do
			Result := tangent_bug_state
		end

	get_pose: POSE_2D
			-- Return current pose.
		do
			Result := current_pose
		end

	get_timestamp: REAL_64
			-- Get timestamp of signaler.
		do
			Result := timestamp
		end

feature -- Status report

	is_go_to_goal: BOOLEAN
			-- Check if state is 'go_to_goal'.
		do
			Result := tangent_bug_state = go_to_goal
		end

	is_follow_wall: BOOLEAN
			-- Check if state is 'follow_wall_cw'.
		do
			Result := tangent_bug_state = follow_wall
		end

	is_leave_wall: BOOLEAN
			-- Check if state is 'leave_wall'.
		do
			Result := tangent_bug_state = leave_wall
		end

	is_at_goal: BOOLEAN
			-- Check if state is 'at_goal'.
		do
			Result := tangent_bug_state = at_goal
		end

	is_unreachable_goal: BOOLEAN
			-- Check if state is 'unreachable_goal'.
		do
			Result := tangent_bug_state = unreachable_goal
		end

feature -- Status setting -- States

	set_at_goal
			-- Set at goal state.
		do
			tangent_bug_state := at_goal
			debug
				io.put_string ("At Goal %N")
			end
		end

	set_follow_wall_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_clockwise
			tangent_bug_state := follow_wall
			debug
				io.put_string ("Follow Wall Clockwise %N")
			end
		end

	set_follow_wall_counter_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_counter_clockwise
			tangent_bug_state := follow_wall
			debug
				io.put_string ("Follow Wall Counter Clockwise%N")
			end
		end

	set_leave_wall_with_target (p: separate POINT_2D)
			-- Set to leave wall state.
		do
			leave_wall.set_target (create {POINT_2D}.make_with_coordinates (p.get_x, p.get_y))
			tangent_bug_state := leave_wall
			debug
				io.put_string ("Leave Wall %N")
			end
		end

	set_go_to_goal
			-- Set to go to goal state.
		do
			tangent_bug_state := go_to_goal
			debug
				io.put_string ("Go to Goal %N")
			end
		end

	set_unreachable_goal
			-- Set to unreachable goal state.
		do
			tangent_bug_state := unreachable_goal
			debug
				io.put_string ("Unreachable Wall %N")
			end
		end

feature -- Status setting

	set_goal (g: POINT_2D)
			-- Set a point as a goal.
		do
			goal := g
		ensure
			goal_set: goal = g
		end

	reset_goal_coordinates (x, y: REAL_64)
			-- Setter for `goal'.
		do
			create goal.make_with_coordinates (x, y)
			set_d_min (goal.get_euclidean_distance (current_pose.get_position))
		end

	set_goal_threshold (threshold: REAL_64)
			-- Set threshold for goal.
		do
			goal_threshold := threshold
		end

	set_d_min (d: REAL_64)
			-- Set minimum distance to goal.
		require
			valid_d: d > 0
		do
			d_min := d
		ensure
			d_set: d = d_min
		end

	set_state (new_state: TANGENT_BUG_STATE)
			-- Set a new state.
		do
			tangent_bug_state := new_state
		ensure
			state_set: tangent_bug_state = new_state
		end

	set_pose (pose: POSE_2D)
			-- Set new pose.
		do
			current_pose := pose
		ensure
			pose_set: current_pose = pose
		end

	set_timestamp (t: REAL_64)
			-- Set time
		do
			timestamp := t
		ensure
			time_set: timestamp = t
		end

feature {NONE} -- Implementation

	goal: POINT_2D
			-- Goal coordinates.

	goal_threshold: REAL_64
			-- Threshold for considering when the goal is reached.

	current_pose: POSE_2D
			-- Present position and orientation.

	timestamp: REAL_64
			-- Time.

	d_min: REAL_64
			-- Minimum distance to goal.

	intial_point_wall: POINT_2D
			-- Position at the beginning of finding a wall.

	tangent_bug_state: TANGENT_BUG_STATE
			-- State of the robot.

	at_goal: AT_GOAL
			-- State "at_goal".

	follow_wall: FOLLOW_WALL
			-- State "follow_wall".

	go_to_goal: GO_TO_GOAL
			-- State "go_to_goal".

	leave_wall: LEAVE_WALL
			-- State "leave_wall".

	unreachable_goal: UNREACHABLE_GOAL
			-- State "unreachable_goal".
end
