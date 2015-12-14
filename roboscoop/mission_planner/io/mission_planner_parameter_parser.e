note
	description: "Main parser for this application."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETER_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature -- Initialization.

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make_default
		end

feature {ANY} -- Access.

	parse_file (file_path: separate STRING)
			-- Parse file with path `file_path'.
		local
			point_array: ARRAYED_LIST[POSE]
			parameters: MISSION_PLANNER_PARAMETERS
			x, y, theta: REAL_64
			file: PLAIN_TEXT_FILE
			frame: STRING
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker
			create point_array.make (0)

			if file_checker.check_file (file) then
				file.open_read
				from
					file.start
				until
					file.off
				loop
					file.read_word_thread_aware
					key := file.last_string.twin

					if key.is_equal ("frame:") then
						file.read_word_thread_aware
						last_parameters.set_frame (file.last_string.twin)
					elseif key.is_equal ("pose:") then
						file.read_double
						x := file.last_double
						file.read_double
						y := file.last_double
						file.read_double
						theta := file.last_double
						point_array.force (create {POSE}.make_with_values (create {POINT}.make_with_values (x, y, 0),
																		   create {QUATERNION}.make_from_heading (theta),
																		   last_parameters.frame))
					elseif key.is_equal ("way_point_threshold:") then
						file.read_double
						last_parameters.set_threshold (file.last_double)
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
				point_array.start
				last_parameters.set_way_points (point_array)
			else
				is_error_found := True
			end
		end

		last_parameters: MISSION_PLANNER_PARAMETERS
				-- Parameters being parsed.
end
