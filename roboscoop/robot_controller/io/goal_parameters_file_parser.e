note
	description: "Goal parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	GOAL_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): GOAL_PARAMETERS
		local
			goal_parameters: GOAL_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create goal_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("X") then
					file.read_double
					goal_parameters.set_x (file.last_double)
				elseif key.is_equal ("Y") then
					file.read_double
					goal_parameters.set_y (file.last_double)
				elseif key.is_equal ("Threshold") then
					file.read_double
					goal_parameters.set_threshold (file.last_double)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := goal_parameters
		end
end
