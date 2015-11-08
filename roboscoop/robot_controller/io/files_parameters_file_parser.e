note
	description: "Files parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	FILES_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): FILES_PARAMETERS
		local
			files_parameters: FILES_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create files_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Goal_parameters_file_path") then
					file.read_word
					files_parameters.set_goal_parameters_file_path (file.last_string)
				elseif key.is_equal ("ROS_topics_file_path") then
					file.read_word
					files_parameters.set_ros_topics_file_path (file.last_string)
				elseif key.is_equal ("PID_parameters_file_path") then
					file.read_word
					files_parameters.set_pid_parameters_file_path (file.last_string)
				elseif key.is_equal ("Wall_following_parameters_file_path") then
					file.read_word
					files_parameters.set_wall_following_parameters_file_path (file.last_string)
				elseif key.is_equal ("Range_sensors_parameters_file_path") then
					file.read_word
					files_parameters.set_range_sensors_parameters_file_path (file.last_string)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := files_parameters
		end
end
