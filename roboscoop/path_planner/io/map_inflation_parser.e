note
	description: "This class parses the map infation parameter."
	author: "Sebastian Curi"
	date: "01.10.2015"

class
	MAP_INFLATION_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			inflation := 0
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get coordinates.
		do
			opened_file.read_character
			opened_file.read_double
			inspect opened_file.last_character
			when 'i' then
				inflation := opened_file.last_double
			else
				io.putstring (opened_file.last_character.out + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
			debug
				io.put_string ("Parsing robot parameters %N")
			end
		end

feature {ANY} -- Access

	inflation: REAL_64
			-- Map Inflation value

end
