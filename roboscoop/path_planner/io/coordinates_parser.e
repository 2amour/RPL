note
	description: "This class parses coordinates from a txt file."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	COORDINATES_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			x := 0
			y := 0
			z := 0
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get coordinates.
		do
			opened_file.read_character
			opened_file.read_double
			inspect opened_file.last_character
			when 'x' then
				x := opened_file.last_double
			when 'y' then
				y := opened_file.last_double
			when 'z' then
				z := opened_file.last_double
			else
				io.putstring (opened_file.last_character.out + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
			debug
				io.put_string ("Parsing coordinates %N")
			end
		end

feature {ANY} -- Access

	x: REAL_64
			-- X coordinate.

	y: REAL_64
			-- Y coordinate.

	z: REAL_64
			-- Z coordinate.

end
