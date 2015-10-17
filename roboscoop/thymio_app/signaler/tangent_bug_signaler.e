note
	description: "State of Tangent bug."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_SIGNALER

create
	make_with_goal

feature{NONE} -- Private Attributes

	goal: POINT_2D
	current_pose: POSE_2D
	timestamp: REAL_64
	d_min: REAL_64
	intial_point_wall: POINT_2D
	state: STATE
	at_goal: AT_GOAL
	follow_wall: FOLLOW_WALL
	go_to_goal: GO_TO_GOAL
	leave_wall: LEAVE_WALL
	unreachable_goal: UNREACHABLE_GOAL

feature --Initialization

	make_with_goal(g: separate POINT_2D)
			-- Init to go_to_goal with goal
		do
			create current_pose.make
			create goal.make_with_coordinates (g.get_x, g.get_y)
			init_states
			set_follow_wall_clockwise
			create intial_point_wall.make
		end

	init_states
			-- initialize states
		do
			create at_goal
			create follow_wall.make_with_v_leave (0.1)
			create go_to_goal.make_with_goal(goal)
			create leave_wall.make_with_v_leave (0.1)
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

	set_leave_wall
			-- Set to leave wall state.
		do
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
			-- Set Goal
		do
			goal := g
		ensure
			goal_set: goal = g
		end

	get_goal: POINT_2D
			-- Get Goal Coordinates
		do
			Result := goal
		end

	set_d_min (d: REAL_64)
			-- Set minimum distance to goal
		require
			valid_d: d>0
		do
			d_min := d
		ensure
			d_set: d = d_min
		end

	get_d_min: REAL_64
			-- Get minimum distance to goal
		do
			Result := d_min
		end

	get_state: STATE
			-- Get Current State
		do
			Result := state
		end

	set_state ( new_state: STATE )
			-- Set a New State
		do
			state := new_state
		ensure
			state_set: state = new_state
		end

	is_go_to_goal: BOOLEAN
			-- Check if state is 'go_to_goal'
		do
			Result := state = go_to_goal
		end

	is_follow_wall: BOOLEAN
			-- Check if state is 'follow_wall_cw'
		do
			Result := state = follow_wall
		end

	is_leave_wall: BOOLEAN
			-- Check if state is 'leave_wall'
		do
			Result := state = leave_wall
		end

	is_at_goal: BOOLEAN
			-- Check if state is 'at_goal'
		do
			Result := state = at_goal
		end

	is_unreachable_goal: BOOLEAN
			-- Check if state is 'unreachable_goal'
		do
			Result := state = unreachable_goal
		end

	get_pose: POSE_2D
			-- Return current pose
		do
			Result := current_pose
		end

	set_pose (pose: POSE_2D)
			-- Set new pose
		do
			current_pose := pose
		ensure
			pose_set: current_pose = pose
		end

	get_timestamp: REAL_64
			-- Get timestamp of signaler
		do
			Result := timestamp
		end

	set_timestamp(t: REAL_64)
		do
			timestamp := t
		ensure
			time_set: timestamp = t
		end


end
