note
	description: "Files parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	FILES_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Implementation

	make
			-- Create Current.
		do
			is_error_found := False
			create last_parameters.make
		end

feature -- Access

	parse_file (file_path: separate STRING)
			-- Parse file.
		local
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				file.open_read
				from file.start
				until file.off
				loop
					file.read_word
					key := file.last_string

					if key.is_equal ("Goal_parameters_file_path") then
						file.read_word
						last_parameters.set_goal_parameters_file_path (file.last_string)
					elseif key.is_equal ("ROS_topics_file_path") then
						file.read_word
						last_parameters.set_ros_topics_file_path (file.last_string)
					elseif key.is_equal ("Go_to_goal_PID_parameters_file_path") then
						file.read_word
						last_parameters.set_go_to_goal_pid_parameters_file_path (file.last_string)
					elseif key.is_equal ("Follow_wall_PID_parameters_file_path") then
						file.read_word
						last_parameters.set_follow_wall_pid_parameters_file_path (file.last_string)
					elseif key.is_equal ("Leave_wall_PID_parameters_file_path") then
						file.read_word
						last_parameters.set_leave_wall_pid_parameters_file_path (file.last_string)
					elseif key.is_equal ("Go_to_goal_non_linear_speed_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_go_to_goal_nlsc_parameters_file_path (file.last_string)
					elseif key.is_equal ("Follow_wall_non_linear_speed_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_follow_wall_nlsc_parameters_file_path (file.last_string)
					elseif key.is_equal ("Leave_wall_non_linear_speed_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_leave_wall_nlsc_parameters_file_path (file.last_string)
					elseif key.is_equal ("Go_to_goal_pose_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_go_to_goal_pose_controller_parameters_file_path (file.last_string)
					elseif key.is_equal ("Follow_wall_pose_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_follow_wall_pose_controller_parameters_file_path (file.last_string)
					elseif key.is_equal ("Leave_wall_pose_controller_parameters_file_path") then
						file.read_word
						last_parameters.set_leave_wall_pose_controller_parameters_file_path (file.last_string)
					elseif key.is_equal ("Wall_following_parameters_file_path") then
						file.read_word
						last_parameters.set_wall_following_parameters_file_path (file.last_string)
					elseif key.is_equal ("Range_sensors_parameters_file_path") then
						file.read_word
						last_parameters.set_range_sensors_parameters_file_path (file.last_string)
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + f_path + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
			else
				is_error_found := True
			end
		end

	last_parameters: FILES_PARAMETERS
			-- Parameters parsed.

end
