note
	description: "This class parses robot parameters."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	ROBOT_PARAMETER_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			r := 0
			v := 0
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get coordinates.
		do
			opened_file.read_character
			opened_file.read_double
			inspect opened_file.last_character
			when 'r' then
				r := opened_file.last_double
			when 'v' then
				v := opened_file.last_double
			else
				io.putstring (opened_file.last_character.out + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
			debug
				io.put_string ("Parsing robot parameters %N")
			end
		end

feature {ANY} -- Access

	r: REAL_64
			-- Robot's radius

	v: REAL_64
			-- Robot's maximum linear velocity

end
