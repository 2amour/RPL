note
	description: "State of Tangent bug."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_SIGNALER

create
	make

feature{NONE} -- Private methods
	state: TANGENT_BUG_STATE

feature --Initialization
	make
		-- Init to go_to_goal
	do
		create state.make_with_state ({TANGENT_BUG_STATE}.go_to_goal)
	end

feature --Accesors

	get_state: TANGENT_BUG_STATE
	do
		Result := state
	end

	set_state ( new_state: TANGENT_BUG_STATE)
	do
		state := new_state
	ensure
		state_set: state = new_state
	end

end
