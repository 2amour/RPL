note
	description: "Non-linear speed controller parameters file parser."
	author: "Ferran Pallarès"
	date: "28.11.15"

class
	NON_LINEAR_SPEED_CONTROLLER_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create -- Implementation
	make

feature

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

			file: PLAIN_TEXT_FILE
			key: STRING
			file_checker: FILE_CHECKER
			f_path: STRING
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

					if key.is_equal ("Maximum_speed") then
						file.read_double
						last_parameters.set_maximum_speed (file.last_double)
					elseif key.is_equal ("Angular_decay_rate") then
						file.read_double
						last_parameters.set_angular_decay_rate (file.last_double)
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

	last_parameters: NON_LINEAR_SPEED_CONTROLLER_PARAMETERS
			-- Parameters parsed.

end
