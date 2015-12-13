note
	description: "Path_planning application root class."
	date: "28.10.2015"
	author: "Sebastian Curi"

class
	APP

inherit
	ROS_ENVIRONMENT
	ARGUMENTS

create
	make

feature {NONE} -- Initialization.

	make
			-- Run application.
		local


			path_planner_node: separate NAMED_ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			mission_behaviour: MISSION_PLANNER_BEHAVIOUR
			mission_signaler: MISSION_PLANNER_SIGNALER
		do
			-- Parse command line arguments.
			if argument_count < 2 then
				io.put_string ("Usage: ./mission_planner mission_planner_parameters_file topics_file%N")
				(create {EXCEPTIONS}).die (-1)
			end

			parse_parameters

			-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NAMED_NODE_STARTER}).roboscoop_node (parameters_bag.mission_planner_topics.node_name)
			synchronize (path_planner_node)

			-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

			-- Start Behaviour.
			create mission_behaviour.make_with_attributes (parameters_bag)
			mission_behaviour.start
		end

feature {NONE} -- Implementation

	parameters: MISSION_PLANNER_PARAMETERS
			-- Parameters for mission planner.

	parameter_parser: MISSION_PLANNER_PARAMETER_PARSER
			-- Mission planner parameters parser.

	topic_parameters: MISSION_PLANNER_TOPICS_PARAMETERS
			-- Parameters for mission planner topics.

	topic_parameters_parser: MISSION_PLANNER_TOPICS_PARSER
			-- Mission planner topics parameters parser.

	parameters_bag: MISSION_PLANNER_PARAMETERS_BAG
			-- Bag of parameters for mission planner.

	parse_parameters
			-- Parse parameters.
		do
			create parameter_parser.make
			parameter_parser.parse_file (argument_array[1])
			if parameter_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				parameters := parameter_parser.last_parameters
			end

			create topic_parameters_parser.make
			topic_parameters_parser.parse_file (argument_array[2])
			if topic_parameters_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				topic_parameters := topic_parameters_parser.last_parameters
			end

			create parameters_bag.make_with_attributes (parameters, topic_parameters)
		end

end
