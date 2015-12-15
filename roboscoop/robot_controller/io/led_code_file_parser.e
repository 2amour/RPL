note
	description: "Led file parser."
	author: "Sebastian Curi"
	date: "15.12.15"

class
	LED_CODE_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Implementation

	make
			-- Create Current.
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
				 file.open_read
				 from file.start
				 until file.off
				 loop
				 	file.read_word
					key := file.last_string
				 	if key.is_equal ("r") then
						file.read_double
						last_parameters.set_r (file.last_double)
					elseif key.is_equal ("g") then
						file.read_double
						last_parameters.set_g (file.last_double)
					elseif key.is_equal ("b") then
						file.read_double
						last_parameters.set_b (file.last_double)
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

	last_parameters: LED_CODE_PARAMETERS
			-- Parameters parsed.
end
