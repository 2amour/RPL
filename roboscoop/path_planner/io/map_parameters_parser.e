note
	description: "This class parses parameters for map processing."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MAP_PARAMETERS_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Initialization.

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
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				file.open_read
				from
					file.start
				until
					file.off
				loop
					file.read_word
					create key.make_from_string (file.last_string)
					if key.is_equal ("block_height:") then
						file.read_integer
						last_parameters.set_block_height (file.last_integer)
					elseif key.is_equal ("block_width:") then
						file.read_integer
						last_parameters.set_block_width (file.last_integer)
					elseif key.is_equal ("inflation:") then
						file.read_double
						last_parameters.set_inflation (file.last_double)
					elseif key.is_equal ("connectivity:") then
						file.read_integer
						inspect file.last_integer
						when 4 then
							last_parameters.set_connectivity_strategy (create {MANHATTAN_CONNECTIVITY_STRATEGY})
						when 8 then
							last_parameters.set_connectivity_strategy (create {FULL_CONNECTIVITY_STRATEGY})
						else
							io.putstring ("Parser error while parsing file '" + file.name + "': Value '" + file.last_integer.out + "' not recognized%N")
							is_error_found := True
						end
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
			else
				is_error_found := True
			end
		end

		last_parameters: MAP_PARAMETERS
end
