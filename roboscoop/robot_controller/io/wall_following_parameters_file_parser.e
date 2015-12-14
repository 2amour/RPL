note
	description: "Wall following parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	WALL_FOLLOWING_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Create Current.
		do
			create last_parameters.make
			is_error_found := False
		end

feature -- Access

	parse_file (file_path: separate STRING)
			-- Parse file.
		local
			x: REAL_64
			y: REAL_64
			file: PLAIN_TEXT_FILE
			key: STRING
			file_checker: FILE_CHECKER
			f_path: STRING
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

					if key.is_equal ("Desired_wall_distance") then
						file.read_double
						last_parameters.set_desired_wall_distance (file.last_double)
					elseif key.is_equal ("Safe_outer_corner_turn_offset") then
						file.read_double
						x := file.last_double
						file.read_double
						y := file.last_double
						last_parameters.set_safe_outer_corner_turn_offset (create {POINT_2D}.make_with_coordinates (x, y))
					elseif key.is_equal ("Safe_leaving_wall_vertical_distance") then
						file.read_double
						last_parameters.set_safe_leaving_wall_vertical_distance (file.last_double)
					elseif key.is_equal ("Safe_outer_corner_turn_offset_threshold") then
						file.read_double
						last_parameters.set_safe_outer_corner_turn_offset_threshold (file.last_double)
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

		last_parameters: WALL_FOLLOWING_PARAMETERS
				-- Parameters parsed.

end
