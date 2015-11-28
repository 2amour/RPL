note
	description: "Non-linear speed controller parameters file parser."
	author: "Ferran Pallarès"
	date: "28.11.15"

class
	NON_LINEAR_SPEED_CONTROLLER_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
		local
			nlsc_parameters: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create nlsc_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Maximum_speed") then
					file.read_double
					nlsc_parameters.set_maximum_speed (file.last_double)
				elseif key.is_equal ("Angular_decay_rate") then
					file.read_double
					nlsc_parameters.set_angular_decay_rate (file.last_double)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := nlsc_parameters
		end
end
