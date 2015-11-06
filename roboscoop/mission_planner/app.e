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
			parameters: MISSION_PLANNER_PARAMETERS
			parameter_parser: MISSION_PLANNER_PARAMETER_PARSER
			topic_parameters: MISSION_PLANNER_TOPICS_PARAMETERS
			topic_parameters_parser: MISSION_PLANNER_TOPICS_PARSER
			parameters_bag: MISSION_PLANNER_PARAMETERS_BAG

			path_planner_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			mission_behaviour: MISSION_PLANNER_BEHAVIOUR
			mission_signaler: MISSION_PLANNER_SIGNALER
		do
				-- Parse command line arguments

			if argument_count < 2 then
				io.put_string ("Usage: ./mission_planner mission_planner_parameters_file topics_file%N")
				(create {EXCEPTIONS}).die (-1)
			end
			create parameter_parser
			parameters := parameter_parser.parse_file (argument_array[1])
			create topic_parameters_parser
			topic_parameters := topic_parameters_parser.parse_file (argument_array[2])
			create parameters_bag.make_with_attributes (parameters, topic_parameters)

				-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (path_planner_node)

				-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

				-- Start Behaviour
			create mission_behaviour.make_with_attributes (parameters_bag)
			mission_behaviour.start
		end

end
