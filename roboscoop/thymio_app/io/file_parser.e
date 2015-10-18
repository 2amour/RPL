note
	description: "Deferred class for parsing key-value structured files."
	author: "Ferran Pallarès"
	date: "06.10.2015"

deferred class
	FILE_PARSER

feature {NONE} -- Initialization

	make
			--Create current.
		deferred
		end

	make_with_path (f_path: STRING)
			-- Create current from given path.
		deferred
		end

feature {NONE} -- Implementation

	file: PLAIN_TEXT_FILE
			-- File to read from.

	file_path: STRING
			-- Path of the file.
end
