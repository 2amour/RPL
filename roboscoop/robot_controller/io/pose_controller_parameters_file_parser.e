note
	description: "Pose controller parameters file parser."
	author: "Ferran Pallarès"
	date: "11.12.15"

class
	POSE_CONTROLLER_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): POSE_CONTROLLER_PARAMETERS
		local
			pose_controller_parameters: POSE_CONTROLLER_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create pose_controller_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Turning_angular_speed") then
					file.read_double
					pose_controller_parameters.set_turning_angular_speed (file.last_double)
				elseif key.is_equal ("Reached_point_threshold") then
					file.read_double
					pose_controller_parameters.set_reached_point_threshold (file.last_double)
				elseif key.is_equal ("Reached_orientation_threshold") then
					file.read_double
					pose_controller_parameters.set_reached_orientation_threshold (file.last_double*{DOUBLE_MATH}.pi/180.0)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := pose_controller_parameters
		end
end
