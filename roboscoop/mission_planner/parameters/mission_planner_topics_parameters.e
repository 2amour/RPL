note
	description: "Mission Planner Topics."
	author: "Sebastian Curi"
	date: "5.11.2015"

class
	MISSION_PLANNER_TOPICS_PARAMETERS

inherit

	TOPIC_PARAMETERS

create
	make_default, make_with_attributes

feature {NONE} -- Initialize

	make_default
			-- Make default
		do
			node_name := "mission_planner_node"
			map := "/map"

			target := "/robot_controller/goal"
			odometry := "/robot_controller/odometry"
			sensed_obstacle := "/robot_controller/obstacles"

			path := "/path_planner/path"
			path_planner_start := "/path_planner/start"
			path_planner_goal := "/path_planner/goal"
			planner_map := "/path_planner/map"

		end

	make_with_attributes (a_name, a_map, a_target, a_odometry, a_obstacle, a_path, a_start, a_goal, b_map: STRING_8)
			-- Create `Current' and assign given attributes.
		do
			node_name := a_name
			map := a_map

			target := a_target
			odometry := a_odometry
			sensed_obstacle := a_obstacle

			path := a_path
			path_planner_start := a_start
			path_planner_goal := a_goal
			planner_map := b_map
		end

feature {ANY} -- Constants

	node_name: STRING_8
			-- Name of node.

	map: STRING_8
			-- Map to be published.

	target: STRING_8
			-- Target goal for the driver.

	odometry: STRING_8
			-- Odometry topic.

	sensed_obstacle: STRING_8
			-- Sensed obstacles topic.

	path: STRING_8
			-- Topic where path will be read.

	path_planner_start: STRING_8
			-- Start position for path_planner node.

	path_planner_goal: STRING_8
			-- Goal position for path_planner node.

	planner_map: STRING_8
			-- Map for path_planner node. 

end
