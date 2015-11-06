note
	description: "Mision planner parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETERS

inherit
	PARAMETERS

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (wp: ARRAYED_LIST[POINT]; threshold: REAL_64)
			-- Create `Current' and assign given attributes.
		do
			way_points := wp
			way_point_threshold := threshold
		end

feature {ANY} -- Acces

	way_points: ARRAYED_LIST[POINT]
			-- way points to visit.

	way_point_threshold: REAL_64
			-- threshold to change way point.

end
