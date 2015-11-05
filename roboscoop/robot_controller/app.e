note
	description: "Example application of Thymio-II in Roboscoop."
	author: "Rusakov Andrey"
	date: "10.09.2014"

class
	APP

inherit
	ROS_ENVIRONMENT

create
	make

feature {NONE} -- Initialization

	make
			-- Create and launch the robot.
		local
			robo_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			tangent_bug_behaviour: separate TANGENT_BUG_BEHAVIOUR
			thymio: separate THYMIO_ROBOT

			-- Execution parameters
			files_params: FILES_PARAMETERS
			files_params_file_parser: FILES_PARAMETERS_FILE_PARSER
			pid_params: PID_PARAMETERS
			pid_params_file_parser: PID_PARAMETERS_FILE_PARSER
			goal_params: GOAL_PARAMETERS
			goal_params_file_parser: GOAL_PARAMETERS_FILE_PARSER
			wall_following_params: WALL_FOLLOWING_PARAMETERS
			wall_following_params_file_parser: WALL_FOLLOWING_PARAMETERS_FILE_PARSER
			range_sensors_params: RANGE_SENSORS_PARAMETERS
			range_sensors_params_file_parser: RANGE_SENSORS_PARAMETERS_FILE_PARSER
			tangent_bug_params: TANGENT_BUG_PARAMETERS_BAG
		do
			-- Check if correct number of command line arguments.
			if Current.arguments.argument_count < 1 then
				io.put_string ("Usage: ./thymio_app file_path%N")
				(create {EXCEPTIONS}).die (-1)
			end

			-- Parse execution parameters
			create files_params_file_parser
			files_params := files_params_file_parser.parse_file (arguments.argument (1).to_string_8)

			create goal_params_file_parser
			goal_params := goal_params_file_parser.parse_file (files_params.goal_parameters_file_path)

			create pid_params_file_parser
			pid_params := pid_params_file_parser.parse_file (files_params.pid_parameters_file_path)

			create wall_following_params_file_parser
			wall_following_params := wall_following_params_file_parser.parse_file (files_params.wall_following_parameters_file_path)

			create range_sensors_params_file_parser
			range_sensors_params := range_sensors_params_file_parser.parse_file (files_params.range_sensors_parameters_file_path)

			create tangent_bug_params.make_with_attributes (goal_params, pid_params, wall_following_params, range_sensors_params)

			-- Initialize this application as a ROS node.
			robo_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (robo_node)

			-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

			-- Initialize behaviour.
			create tangent_bug_behaviour.make_with_attributes (tangent_bug_params)

			-- Create a robot object.
			create thymio.make_with_attributes (range_sensors_params, tangent_bug_behaviour)

			-- Launch Thymio.
			separate thymio as t do
				t.dispatch
			end
		end
end
