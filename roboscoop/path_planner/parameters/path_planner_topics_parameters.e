note
	description: "Path planner topics."
	author: "Sebastian Curi"
	date: "5.11.2015"
	revision: "$Revision$"

class
	PATH_PLANNER_TOPICS_PARAMETERS

inherit

	TOPIC_PARAMETERS

create
	make_default, make_with_attributes

feature {NONE} -- Initialize

	make_default
			-- Make default
		do
			map := "/map"
			path := "/path_planner/path"
			start := "/path_planner/start"
			goal := "/path_planner/goal"
		end

	make_with_attributes (a_map, a_path, a_start, a_goal: STRING_8)
			-- Create `Current' and assign given attributes.
		do
			map := a_map
			path := a_path
			start := a_start
			goal := a_goal
		end


feature {ANY} -- Access

	map: STRING_8
			-- map to be read.

	path: STRING_8
			-- topic where path must be published.

	start: STRING_8
			-- topic where start position must be read.

	goal: STRING_8
			-- topic where goal position must be read.

end
