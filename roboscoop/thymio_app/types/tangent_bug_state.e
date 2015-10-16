note
	description: "Pseudo ENUM Simulation of the TANGENT BUG STATES."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_STATE

create
	make_with_state

feature -- Initializors
	make_with_state(i: INTEGER)
	require
		is_valid:   i = go_to_goal or
				    i = follow_wall or
				    i = leave_wall or
				    i = at_goal or
				    i = unreachable_goal
	do
		value := i
	ensure
		value_set: value = i
	end
feature --Accesors
	set_state ( new_state: INTEGER )
	do
		make_with_state (new_state)
	end

	get_state: INTEGER
	do
		Result := value
	end


feature {NONE} -- Attributes
	value: INTEGER
feature{ANY}
	go_to_goal: INTEGER = 1
	follow_wall: INTEGER = 2
	leave_wall: INTEGER = 3
	at_goal: INTEGER = 4
	unreachable_goal: INTEGER = 5



invariant
	value_in_range: value = go_to_goal or
				    value = follow_wall or
				    value = leave_wall or
				    value = at_goal or
				    value = unreachable_goal


end
