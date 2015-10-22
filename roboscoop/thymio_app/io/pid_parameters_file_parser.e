note
	description: "PID parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	PID_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

feature -- Access

	parse_file (file_path: STRING): PID_PARAMETERS
		local
			pid_parameters: PID_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create pid_parameters.make
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_word
				key := file.last_string

				if key.is_equal ("Kp") then
					file.read_double
					pid_parameters.set_kp (file.last_double)
				elseif key.is_equal ("Ki") then
					file.read_double
					pid_parameters.set_ki (file.last_double)
				elseif key.is_equal ("Kd") then
					file.read_double
					pid_parameters.set_kd (file.last_double)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end

			file.close
			Result := pid_parameters
		end
end
