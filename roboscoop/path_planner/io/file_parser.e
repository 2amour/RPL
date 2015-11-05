note
	description: "General class from parsing from a txt file."
	author: "Sebastian Curi"
	date: "31.10.2015"

deferred class
	FILE_PARSER

feature {ANY} -- Access

	parse_path (f_path: STRING)
			-- Create current from given path.
		do
			file_path := f_path
			create file.make_open_read (file_path)
			from
				file.start
			until
				file.off
			loop
				parse (file)
			end
			file.close
		end

feature {NONE} -- Implementation

	exception: EXCEPTIONS
			-- Exception handling mechanism

	file: PLAIN_TEXT_FILE
			-- File to read from.

	file_path: STRING
			-- Path of the file.

	parse (opened_file: PLAIN_TEXT_FILE)
			-- The parser to be implemented
		deferred
		end

	make_default
			-- Make default params
		do
			create exception
			file_path := " "
			file := create {PLAIN_TEXT_FILE}.make_with_name (file_path)
		end

end
