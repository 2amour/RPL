note
	description: "Class for parsing key-value structured files."
	author: "Ferran Pallarès"
	date: "06.10.2015"

deferred class
	FILE_PARSER

feature {NONE} -- Initialization

	make_with_path (path: STRING)
			-- Create current
		deferred
		end

feature {NONE} -- Implementation

	file: PLAIN_TEXT_FILE
	file_path: STRING

end
