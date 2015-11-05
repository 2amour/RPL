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
			mission_behaviour: MISSION_PLANNER_BEHAVIOUR
			mission_signaler: MISSION_PLANNER_SIGNALER
		do
				-- Parse command line arguments
			create parser.make
			parser.parse_args (argument_count, argument_array)

				-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (path_planner_node)

				-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

				-- Start Signalers
			create mission_signaler.make_with_attributes (parser.way_points_parser.points.first, parser.way_points_parser.points.last, 0.05) -- TODO
				-- Start Behaviour
			create mission_behaviour.make_with_attributes (mission_signaler)
			mission_behaviour.start
		end

end
