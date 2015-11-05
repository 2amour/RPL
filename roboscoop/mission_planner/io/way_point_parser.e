note
	description: "This class parses a list of points."
	author: "Sebastian Curi"
	date: "02.11.2015"

class
	WAY_POINT_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			create points.make
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get coordinates.
		local
			new_file: PLAIN_TEXT_FILE
				-- File to read from.

			new_path: STRING
				-- Path of the file.

			coordinates_parser: COORDINATES_PARSER
			-- Parser for coordiates

		do
			opened_file.read_line
			new_path := "auxiliar.txt"
			create new_file.make_create_read_write (new_path)
			new_file.putstring (opened_file.last_string)
			new_file.close
			create coordinates_parser.make
			coordinates_parser.parse_path (new_path)
			points.put_right (create {POINT}.make_with_values (coordinates_parser.x, coordinates_parser.y, coordinates_parser.z))
			if opened_file.end_of_file then
				new_file.delete -- Delete auxiliar file.
			end
		end

feature {ANY} -- Access

	points: LINKED_LIST [POINT]
			-- Array of points.

end
