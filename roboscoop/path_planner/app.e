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
			algorithm_parameters: PATH_PLANNER_PARAMETERS
			algorithm_parameter_parser: PATH_PLANNER_PARAMETERS_PARSER

			map_parameters: MAP_PARAMETERS
			map_parameter_parser: MAP_PARAMETERS_PARSER

			topic_parameters: PATH_PLANNER_TOPICS_PARAMETERS
			topic_parameters_parser: PATH_PLANNER_TOPICS_PARSER
			parameters_bag: PATH_PLANNER_PARAMETERS_BAG


			path_planner_node: separate NAMED_ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER

			path_planning_behaviour: PATH_PLANNING_BEHAVIOUR

		do
				-- Parse command line arguments
			if argument_count < 3 then
				io.put_string ("Usage: ./path_planner algorithm_parameters_file map_parameters_file topics_file%N")
				(create {EXCEPTIONS}).die (-1)
			end

			create algorithm_parameter_parser
			algorithm_parameters := algorithm_parameter_parser.parse_file (argument_array[1])

			create map_parameter_parser
			map_parameters := map_parameter_parser.parse_file (argument_array[2])

			create topic_parameters_parser
			topic_parameters := topic_parameters_parser.parse_file (argument_array[3])

			create parameters_bag.make_with_attributes (map_parameters, algorithm_parameters, topic_parameters)


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

end
