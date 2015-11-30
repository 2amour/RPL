note
	description: " Path_planning application root class."
	author: "Antoni Rosinol"
	date: "28.10.2015"

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

			path_planning_behaviour: PATH_PLANNING_BEHAVIOUR
		do
				-- Parse command line arguments.
			if argument_count < 3 then
				io.put_string ("Usage: ./path_planner algorithm_parameters_file map_parameters_file topics_file%N")
				(create {EXCEPTIONS}).die (-1)
			end

				-- Parse parameters.
			parse_parameters

				-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NAMED_NODE_STARTER}).roboscoop_node (parameters_bag.path_planner_topics.node_name)
			synchronize (path_planner_node)

				-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

				-- Start Behaviour
			create path_planning_behaviour.make_with_attributes (parameters_bag)
			path_planning_behaviour.start
		end

feature {NONE} -- Implementation

	algorithm_parameters: PATH_PLANNER_PARAMETERS
			-- Parameters for path planning.

	algorithm_parameter_parser: PATH_PLANNER_PARAMETERS_PARSER
			-- Parser for path planning parameters

	map_parameters: MAP_PARAMETERS
			-- Parameters of the map.

	map_parameter_parser: MAP_PARAMETERS_PARSER
			-- Parser for the parameters of the map.

	topic_parameters: PATH_PLANNER_TOPICS_PARAMETERS
			-- Parameters of the topics for path planning.

	topic_parameters_parser: PATH_PLANNER_TOPICS_PARSER
			-- Parser for the parameters of the topics for path planning.

	parameters_bag: PATH_PLANNER_PARAMETERS_BAG
			-- Bag of parameters for path planning.

	parse_parameters
			-- Parse set of parameters
		do
			create algorithm_parameter_parser.make
			algorithm_parameter_parser.parse_file (argument_array[1])
			if algorithm_parameter_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				algorithm_parameters := algorithm_parameter_parser.last_parameters
			end

			create map_parameter_parser.make
			map_parameter_parser.parse_file (argument_array[2])
			if map_parameter_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				map_parameters := map_parameter_parser.last_parameters
			end

			create topic_parameters_parser.make
			topic_parameters_parser.parse_file (argument_array[3])
			if topic_parameters_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				topic_parameters := topic_parameters_parser.last_parameters
			end

			create parameters_bag.make_with_attributes (map_parameters, algorithm_parameters, topic_parameters)
		end

end
