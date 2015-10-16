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
		create goal.make_with_coordinates (g.get_x, g.get_y)
		init_states
		state := go_to_goal
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
		end

	set_follow_wall
			-- Set follow wall state.
		do
			state := follow_wall
		end

	set_leave_wall
			-- Set to leave wall state.
		do
			state := leave_wall
		end

	set_go_to_goal
			-- Set to go to goal state.
		do
			state := go_to_goal
		end

	set_unreachable_goal
			-- Set to unreachable goal state.
		do
			state := unreachable_goal
		end

	set_goal (g: POINT_2D)
			-- Set Goal
		do
			goal := g
		end

	get_goal: POINT_2D
		-- Get Goal Coordinates
	do
		Result := goal
	end

	set_d_min(d: REAL_64)
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
		-- Check if state is 'follow_wall'
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


end
