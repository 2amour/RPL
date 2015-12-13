note
	description: "Pose controller parameters file parser."
	author: "Ferran Pallarès"
	date: "11.12.15"

class
	POSE_CONTROLLER_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature -- Initialization

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make
		end

feature -- Access

	parse_file (file_path: separate STRING)
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
				from file.start
				until file.off
				loop
					file.read_word
					key := file.last_string

					if key.is_equal ("Turning_angular_speed") then
						file.read_double
						last_parameters.set_turning_angular_speed (file.last_double)
					elseif key.is_equal ("Reached_point_threshold") then
						file.read_double
						last_parameters.set_reached_point_threshold (file.last_double)
					elseif key.is_equal ("Reached_orientation_threshold") then
						file.read_double
						last_parameters.set_reached_orientation_threshold (file.last_double*{DOUBLE_MATH}.pi/180.0)
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + f_path + "': Key '" + key + "' not recognized%N")
					end
				end
				file.close
			else
				is_error_found := True
			end
		end

		last_parameters: POSE_CONTROLLER_PARAMETERS
end
