note
	description: "PID parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

class
	PID_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature -- Initialization

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make
		end

feature -- Access

	parse_file (file_path: separate STRING)
		local
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
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

					if key.is_equal ("Kp") then
						file.read_double
						last_parameters.set_kp (file.last_double)
					elseif key.is_equal ("Ki") then
						file.read_double
						last_parameters.set_ki (file.last_double)
					elseif key.is_equal ("Kd") then
						file.read_double
						last_parameters.set_kd (file.last_double)
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

		last_parameters: PID_PARAMETERS
end
