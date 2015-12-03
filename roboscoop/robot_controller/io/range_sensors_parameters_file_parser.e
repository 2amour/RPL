note
	description: "Range sensors parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	RANGE_SENSORS_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make
		end

feature -- Access

	parse_file (file_path: separate STRING)
			-- Parse file.
		local
			sensor_count: INTEGER_32
			x, y, phi: REAL_64
			sensors_poses: ARRAY[POSE_2D]
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			create sensors_poses.make_empty
			create last_parameters.make
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				from file.start
				until file.off
				loop
					file.read_word
					key := file.last_string

					if key.is_equal ("Sensor_count") then
						file.read_integer
						sensor_count := file.last_integer
						create sensors_poses.make_filled (create {POSE_2D}.make, 1, sensor_count)
					elseif key.is_equal ("Close_obstacle_threshold") then
										file.read_double
										last_parameters.set_close_obstacle_threshold (file.last_double)
					elseif key.is_equal ("Sensors_poses") then
						from until sensor_count <= 0
						loop
							file.read_double
							x := file.last_double
							file.read_double
							y := file.last_double
							file.read_double
							phi := file.last_double

							sensor_count := sensor_count - 1
							sensors_poses.put (create {POSE_2D}.make_with_coordinates (x, y, phi), sensors_poses.upper - sensor_count)
						end
						last_parameters.set_sensors_poses (sensors_poses)
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

		last_parameters: RANGE_SENSORS_PARAMETERS
				-- Parameters parsed.
end
