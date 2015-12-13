note
	description: "Example application of Thymio-II in Roboscoop."
	author: "Rusakov Andrey"
	date: "10.09.2014"

class
	APP

inherit
	ROS_ENVIRONMENT
	ARGUMENTS

create
	make

feature {NONE} -- Initialization

	make
			-- Create and launch the robot.
		local
			robo_node: separate NAMED_ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			tangent_bug_behaviour: separate TANGENT_BUG_BEHAVIOUR
			led_behaviour: separate OBJECT_DISPLAY_BEHAVIOUR
			thymio: separate THYMIO_ROBOT
		do
			-- Check if correct number of command line arguments.
			if argument_count < 1 then
				io.put_string ("Usage: ./thymio_app file_path%N")
				(create {EXCEPTIONS}).die (-1)
			end

			-- Parse execution parameters
			parse_parameters

			-- Initialize this application as a ROS node.
			robo_node := (create {ROS_NAMED_NODE_STARTER}).roboscoop_node (topics.name)
			synchronize (robo_node)

			-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

			-- Initialize behaviour.
			create tangent_bug_behaviour.make_with_attributes (topics, tangent_bug_params)
			create led_behaviour.make_with_attributes (topics)

			-- Create a robot object.
			create thymio.make_with_attributes (range_sensors_params)

			-- Set robot behaviour
			set_robot_behaviour (thymio, tangent_bug_behaviour)

			separate led_behaviour as ld do
				ld.start
			end
			-- Launch Thymio.
			separate thymio as t do
				t.dispatch
			end
		end

feature {NONE} -- Implementation

	set_robot_behaviour (robot: separate THYMIO_ROBOT; behaviour: separate ROBOT_BEHAVIOUR)
			-- Set a thymio robot's behaviour.
		do
			robot.set_behaviour(behaviour)
		end

	topics: ROBOT_CONTROLLER_TOPIC_PARAMETERS
			-- Parameters for topics for the robot controller.

	topics_parser: ROBOT_CONTROLLER_TOPICS_PARSER
			-- Parser for the parameters for topics for the robot controller.

	files_params: FILES_PARAMETERS
			-- Parameters for the paths of the files with parameters.

	files_params_file_parser: FILES_PARAMETERS_FILE_PARSER
			-- Parser for the parameters for the paths of the files with parameters.

	gtg_pid_params: PID_PARAMETERS
			-- PID parameters for the go to goal state.

	fw_pid_params: PID_PARAMETERS
			-- PID parameters for the follow wall state.

	lw_pid_params: PID_PARAMETERS
			-- PID parameters for the leave wall state.

	pid_params_file_parser: PID_PARAMETERS_FILE_PARSER
			-- Parser for the PID parameters.

	gtg_nlsc_params: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			-- Non linear speed controller parameters for the go to goal state.

	fw_nlsc_params: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			-- Non linear speed controller parameters for the follow wall state.

	lw_nlsc_params: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			-- Non linear speed controller parameters for the leave wall state.

	nlsc_params_file_parser: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS_FILE_PARSER
			-- Parser for non linear speed controller parameters.

	gtg_pose_controller_params: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for the go to goal state.

	fw_pose_controller_params: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for the follow wall state.

	lw_pose_controller_params: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for the leave wall state.

	pose_controller_params_file_parser: POSE_CONTROLLER_PARAMETERS_FILE_PARSER
			-- Parser for pose controller parameters.

	goal_params: GOAL_PARAMETERS
			-- Goal parameters.

	goal_params_file_parser: GOAL_PARAMETERS_FILE_PARSER
			-- Parser for the goal parameters.

	wall_following_params: WALL_FOLLOWING_PARAMETERS
			-- Parameters for the wall_following state.

	wall_following_params_file_parser: WALL_FOLLOWING_PARAMETERS_FILE_PARSER
			-- Parser for the wall_following state.

	range_sensors_params: RANGE_SENSORS_PARAMETERS
			-- Range sensor parameters.

	range_sensors_params_file_parser: RANGE_SENSORS_PARAMETERS_FILE_PARSER
			-- Parser for range sensor parameters.

	tangent_bug_params: TANGENT_BUG_PARAMETERS_BAG
			-- Parameters bag for tangent bug algorithm.

	parse_parameters
			-- Parse set of parameters.
		do
			create files_params_file_parser.make
			files_params_file_parser.parse_file (arguments.argument (1).to_string_8)
			if files_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				files_params := files_params_file_parser.last_parameters
			end

			create topics_parser.make
			topics_parser.parse_file (files_params.ros_topics_file_path)
			if topics_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				topics := topics_parser.last_parameters
			end

			create goal_params_file_parser.make
			goal_params_file_parser.parse_file (files_params.goal_parameters_file_path)
			if goal_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				goal_params := goal_params_file_parser.last_parameters
			end

			create pid_params_file_parser.make
			pid_params_file_parser.parse_file (files_params.go_to_goal_pid_parameters_file_path)
			if pid_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				gtg_pid_params := pid_params_file_parser.last_parameters
			end

			pid_params_file_parser.parse_file (files_params.follow_wall_pid_parameters_file_path)
			if pid_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				fw_pid_params := pid_params_file_parser.last_parameters
			end

			pid_params_file_parser.parse_file (files_params.leave_wall_pid_parameters_file_path)
			if pid_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				lw_pid_params := pid_params_file_parser.last_parameters
			end

			create nlsc_params_file_parser.make
			nlsc_params_file_parser.parse_file (files_params.go_to_goal_nlsc_parameters_file_path)
			if nlsc_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				gtg_nlsc_params := nlsc_params_file_parser.last_parameters
			end

			nlsc_params_file_parser.parse_file (files_params.follow_wall_nlsc_parameters_file_path)
			if nlsc_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				fw_nlsc_params := nlsc_params_file_parser.last_parameters
			end

			nlsc_params_file_parser.parse_file (files_params.leave_wall_nlsc_parameters_file_path)
			if nlsc_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				lw_nlsc_params := nlsc_params_file_parser.last_parameters
			end

			create pose_controller_params_file_parser.make
			pose_controller_params_file_parser.parse_file (files_params.go_to_goal_pose_controller_parameters_file_path)
			if pose_controller_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				gtg_pose_controller_params := pose_controller_params_file_parser.last_parameters
				gtg_pose_controller_params.set_pid_parameters (gtg_pid_params)
				gtg_pose_controller_params.set_nlsc_parameters (gtg_nlsc_params)
			end

			pose_controller_params_file_parser.parse_file (files_params.follow_wall_pose_controller_parameters_file_path)
			if pose_controller_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				fw_pose_controller_params := pose_controller_params_file_parser.last_parameters
				fw_pose_controller_params.set_pid_parameters (fw_pid_params)
				fw_pose_controller_params.set_nlsc_parameters (fw_nlsc_params)
			end

			pose_controller_params_file_parser.parse_file (files_params.follow_wall_pose_controller_parameters_file_path)
			if pose_controller_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				lw_pose_controller_params := pose_controller_params_file_parser.last_parameters
				lw_pose_controller_params.set_pid_parameters (lw_pid_params)
				lw_pose_controller_params.set_nlsc_parameters (lw_nlsc_params)
			end

			create wall_following_params_file_parser.make
			wall_following_params_file_parser.parse_file (files_params.wall_following_parameters_file_path)
			if wall_following_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				wall_following_params := wall_following_params_file_parser.last_parameters
			end

			create range_sensors_params_file_parser.make
			range_sensors_params_file_parser.parse_file (files_params.range_sensors_parameters_file_path)
			if range_sensors_params_file_parser.is_error_found then
				(create {EXCEPTIONS}).die (-1)
			else
				range_sensors_params :=  range_sensors_params_file_parser.last_parameters
			end

			create tangent_bug_params.make_with_attributes (goal_params, wall_following_params, gtg_pose_controller_params, fw_pose_controller_params, lw_pose_controller_params)
		end
end
