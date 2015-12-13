note
	description: "Parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

deferred class
	PARAMETERS_FILE_PARSER

feature {ANY} -- Access

	parse_file (file_path: separate STRING)
			-- Parse file with path `file_path'.
		deferred
		end

	is_error_found: BOOLEAN

end
