note
	description: "Main parser for this application."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETER_PARSER

inherit

	PARAMETERS_FILE_PARSER


feature {ANY} -- Acces.

	parse_file (file_path: separate STRING): MISSION_PLANNER_PARAMETERS
			-- Parse file with path `file_path'.
		local
			point_array: ARRAYED_LIST[POSE]
			parameters: MISSION_PLANNER_PARAMETERS
			x, y, theta: REAL_64
			file: PLAIN_TEXT_FILE
			frame: STRING
			key: STRING
		do
			create parameters.make_default

			create point_array.make (0)
			create file.make_open_read (create {STRING}.make_from_separate (file_path))
			from
				file.start
			until
				file.off
			loop
				file.read_word
				frame := ""
				key := file.last_string

				if key.is_equal ("frame: ") then
					file.read_word_thread_aware
					frame := file.last_string
					parameters.set_frame (frame)
				elseif key.is_equal ("pose:") then
					file.read_double
					x := file.last_double
					file.read_double
					y := file.last_double
					file.read_double
					theta := file.last_double
					point_array.force (create {POSE}.make_with_values (create {POINT}.make_with_values (x, y, 0),
																	   create {QUATERNION}.make_from_heading (theta),
																	   frame))
				elseif key.is_equal ("way_point_threshold:") then
					file.read_double
					parameters.set_threshold (file.last_double)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close
			point_array.start

			parameters.set_way_points (point_array)
			Result := parameters
		end

end
