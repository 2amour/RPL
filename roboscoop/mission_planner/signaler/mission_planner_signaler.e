note
	description: "Signaler with higher-level mission parameters."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_start, a_goal, a_target: POINT)
			-- Make `Current' and assign its attributes.
		do
			start := a_start
			goal := a_goal
			target := a_target
		end

feature {ANY} -- Access

	start: POINT
			-- Starting point.

	goal: POINT
			-- Goal point.

	target: POINT
			-- Current target point.

end
