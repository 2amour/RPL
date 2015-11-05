note
	description: "path_planning application root class"
	date: "28.10.2015"

class
	APP

inherit

	ROS_ENVIRONMENT

	ARGUMENTS

create
	make

feature {NONE} -- Initialization

	make
			-- Run application.
		local
			parser: MISSION_PLANNER_PARSER
			path_planner_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
		do
				-- Parse command line arguments
			create parser.make
			--parser.way_points_parser.points

				-- Transform LIST[POINT_MSGS] to LIST[SPATIAL_GRAPH_NODES]

				-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (path_planner_node)

				-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

				-- Start Signalers

				-- Start Behaviour

		end

end
