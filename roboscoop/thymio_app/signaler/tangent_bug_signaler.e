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

	state: TANGENT_BUG_STATE
	d_min: REAL_64
	intial_point_wall: POINT_2D
	goal: POINT_2D

feature --Initialization
	make_with_goal(g: POINT_2D)
		-- Init to go_to_goal with goal
	do
		create goal.make_with_coordinates (g.get_x, g.get_y)
		create state.make_with_state ({TANGENT_BUG_STATE}.go_to_goal)
		create intial_point_wall.make
	end

feature --Accesors
	set_goal(g: POINT_2D)
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

	get_state: TANGENT_BUG_STATE
		-- Get Current State
	do
		Result := state
	end

	set_state ( new_state: TANGENT_BUG_STATE)
		-- Set a New State
	do
		state := new_state
	ensure
		state_set: state = new_state
	end

	is_go_to_goal: BOOLEAN
		-- Check if state is 'go_to_goal'
	do
		Result := state.get_state = {TANGENT_BUG_STATE}.go_to_goal
	end

	is_follow_wall: BOOLEAN
		-- Check if state is 'follow_wall'
	do
		Result := state.get_state = {TANGENT_BUG_STATE}.follow_wall
	end

	is_leave_wall: BOOLEAN
		-- Check if state is 'leave_wall'
	do
		Result := state.get_state = {TANGENT_BUG_STATE}.leave_wall
	end

	is_at_goal: BOOLEAN
		-- Check if state is 'at_goal'
	do
		Result := state.get_state = {TANGENT_BUG_STATE}.at_goal
	end

	is_unreachable_goal: BOOLEAN
		-- Check if state is 'unreachable_goal'
	do
		Result := state.get_state = {TANGENT_BUG_STATE}.unreachable_goal
	end


end
