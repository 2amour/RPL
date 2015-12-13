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
	make_default

feature {NONE} -- Initialize

	make_default
			-- Make default
		do
			node_name := "path_planning_node"
			map := "/map"
			path := "/path_planner/path"
			start := "/path_planner/start"
			goal := "/path_planner/goal"
		end

feature {ANY} -- Access

	node_name: STRING
			-- Name of node.

	set_node_name (a_name: separate STRING)
			-- Set the node name.
		do
			node_name := create{STRING}.make_from_separate (a_name)
		end

	map: STRING
			-- Map to be read.

	set_map (a_map: separate STRING)
			-- Set the node name.
		do
			map := create{STRING}.make_from_separate (a_map)
		end

	path: STRING
			-- Topic where path must be published.

	set_path (a_path: separate STRING)
			-- Set the path topic.
		do
			path := create{STRING}.make_from_separate (a_path)
		end

	start: STRING
			-- Topic where start position must be read.

	set_start (a_start: separate STRING)
			-- Set the start topic.
		do
			start := create{STRING}.make_from_separate (a_start)
		end

	goal: STRING
			-- Topic where goal position must be read.

	set_goal (a_goal: separate STRING)
			-- Set the goal topic.
		do
			goal := create{STRING}.make_from_separate (a_goal)
		end

end
