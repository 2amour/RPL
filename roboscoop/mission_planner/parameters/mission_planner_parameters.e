note
	description: "Mision planner parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETERS

inherit
	PARAMETERS

create
	make_default --, make_with_attributes

feature {NONE} -- Initialization

	make_default
			-- Create `Current' with default values
		do
			create way_points.make (0)
		end

--	make_with_attributes (wp: ARRAYED_LIST[POINT]; threshold: REAL_64)
--			-- Create `Current' and assign given attributes.
--		do
--			way_points := wp
--			way_point_threshold := threshold
--		end

feature {ANY} -- Acces

	way_points: ARRAYED_LIST[POINT]
			-- Way points to visit.

	set_way_points (wp: ARRAYED_LIST[POINT])
			-- Set the way points.
		do
			way_points := wp
		end

	way_point_threshold: REAL_64
			-- Threshold to change way point.

	set_threshold (threshold: REAL_64)
			-- Set threhsold to change way point.
		do
			way_point_threshold := threshold
		end

end
