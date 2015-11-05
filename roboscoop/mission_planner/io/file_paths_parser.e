note
	description: "This class parses the files paths from a file."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	FILE_PATHS_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			create file_paths.make (0)
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get graph search paameters.
		do
			opened_file.read_word_thread_aware
			file_paths.put (create {STRING_8}.make_from_string (opened_file.last_string))
			debug
				io.put_string (opened_file.last_string + "%N")
			end
		end

feature {ANY} -- Access

	file_paths: ARRAYED_QUEUE [STRING_8]
			-- Array with paths.

end
