note
	description: "Range sensors parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	RANGE_SENSORS_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): RANGE_SENSORS_PARAMETERS
		local
			sensor_count: INTEGER_32
			x, y, phi: REAL_64
			sensors_poses: ARRAY[POSE_2D]
			range_sensors_parameters: RANGE_SENSORS_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create sensors_poses.make_empty
			create range_sensors_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Sensor_count") then
					file.read_integer
					sensor_count := file.last_integer
					create sensors_poses.make_filled (create {POSE_2D}.make, 1, sensor_count)
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
					range_sensors_parameters.set_sensors_poses (sensors_poses)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := range_sensors_parameters
		end
end
