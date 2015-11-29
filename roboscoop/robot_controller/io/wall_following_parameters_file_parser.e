note
	description: "Wall following parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	WALL_FOLLOWING_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): WALL_FOLLOWING_PARAMETERS
		local
			x: REAL_64
			y: REAL_64
			wall_following_parameters: WALL_FOLLOWING_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create wall_following_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Desired_wall_distance") then
					file.read_double
					wall_following_parameters.set_desired_wall_distance (file.last_double)
				elseif key.is_equal ("Outer_corner_angular_velocity") then
					file.read_double
					wall_following_parameters.set_outer_corner_angular_velocity (file.last_double)
				elseif key.is_equal ("Safe_outer_corner_turn_offset") then
					file.read_double
					x := file.last_double
					file.read_double
					y := file.last_double
					wall_following_parameters.set_safe_outer_corner_turn_offset (create {POINT_2D}.make_with_coordinates (x, y))
				elseif key.is_equal ("Safe_leaving_wall_vertical_distance") then
					file.read_double
					wall_following_parameters.set_safe_leaving_wall_vertical_distance (file.last_double)
				elseif key.is_equal ("Safe_outer_corner_turn_offset_threshold") then
					file.read_double
					wall_following_parameters.set_safe_outer_corner_turn_offset_threshold (file.last_double)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := wall_following_parameters
		end
end
