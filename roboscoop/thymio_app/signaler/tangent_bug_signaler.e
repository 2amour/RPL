note
	description: "State of Tangent bug."
	author: "ferran_antoni_sebastian"
	date: "18.10.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_goal

feature {NONE} -- Initialization

	make_with_goal (g: separate POINT_2D)
			-- Initialize to go_to_goal with goal.
		do
			create current_pose.make
			create goal.make_with_coordinates (g.get_x, g.get_y)
			init_states
			set_follow_wall_counter_clockwise
--			set_follow_wall_clockwise
			create intial_point_wall.make
		end

	init_states
			-- Initialize states.
		do
			create at_goal
			create follow_wall.make --.make_with_v_leave (0.1)
			create go_to_goal.make
			create leave_wall.make -- TODO - .make_with_v_leave (0.1)
			create unreachable_goal
		end

feature -- Access

	set_at_goal
			-- Set at goal state.
		do
			state := at_goal
			debug
				io.put_string ("At Goal %N")
			end
		end

	set_follow_wall_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_clockwise
			state := follow_wall
			debug
				io.put_string ("Follow Wall Clockwise %N")
			end
		end

	set_follow_wall_counter_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_counter_clockwise
			state := follow_wall
			debug
				io.put_string ("Follow Wall Counter Clockwise%N")
			end
		end

	set_leave_wall -- TODO DELETE THIS
			-- Set to leave wall state.
		do
			state := leave_wall
			debug
				io.put_string ("Leave Wall %N")
			end
		end

	set_leave_wall_with_target (p: POINT_2D)
			-- Set to leave wall state.
		do
			leave_wall.set_target (p)
			state := leave_wall
			debug
				io.put_string ("Leave Wall %N")
			end
		end

	set_go_to_goal
			-- Set to go to goal state.
		do
			state := go_to_goal
			debug
				io.put_string ("Go to Goal %N")
			end
		end

	set_unreachable_goal
			-- Set to unreachable goal state.
		do
			state := unreachable_goal
			debug
				io.put_string ("Unreachable Wall %N")
			end
		end

	set_goal (g: POINT_2D)
			-- Set a point as a goal.
		do
			goal := g
		ensure
			goal_set: goal = g
		end

	get_goal: POINT_2D
			-- Get goal coordinates.
		do
			Result := goal
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

	get_d_min: REAL_64
			-- Get minimum distance to goal.
		do
			Result := d_min
		end

	get_state: STATE
			-- Get current state.
		do
			Result := state
		end

	set_state (new_state: STATE)
			-- Set a new state.
		do
			state := new_state
		ensure
			state_set: state = new_state
		end

	is_go_to_goal: BOOLEAN
			-- Check if state is 'go_to_goal'.
		do
			Result := state = go_to_goal
		end

	is_follow_wall: BOOLEAN
			-- Check if state is 'follow_wall_cw'.
		do
			Result := state = follow_wall
		end

	is_leave_wall: BOOLEAN
			-- Check if state is 'leave_wall'.
		do
			Result := state = leave_wall
		end

	is_at_goal: BOOLEAN
			-- Check if state is 'at_goal'.
		do
			Result := state = at_goal
		end

	is_unreachable_goal: BOOLEAN
			-- Check if state is 'unreachable_goal'.
		do
			Result := state = unreachable_goal
		end

	get_pose: POSE_2D
			-- Return current pose.
		do
			Result := current_pose
		end

	set_pose (pose: POSE_2D)
			-- Set new pose.
		do
			current_pose := pose
		ensure
			pose_set: current_pose = pose
		end

	get_timestamp: REAL_64
			-- Get timestamp of signaler.
		do
			Result := timestamp
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

	current_pose: POSE_2D
			-- Present position and orientation.

	timestamp: REAL_64
			-- Time.

	d_min: REAL_64
			-- Minimum distance to goal.

	intial_point_wall: POINT_2D
			-- Position at the beginning of finding a wall.

	state: STATE
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
