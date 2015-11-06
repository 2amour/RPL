note
	description: "Parameters file parser."
	author: "Ferran Pallarès"
	date: "21.10.15"

deferred class
	PARAMETERS_FILE_PARSER

feature {ANY} -- Access

	parse_file (file_path: STRING): PARAMETERS
			-- Parse file with path `file_path'.
		deferred
		end
		
end
