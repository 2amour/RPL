note
	description: "This class parses parameters for map processing."
	author: "Sebastian Curi"
	date: "06.11.2015"


class
	MAP_PARAMETERS_PARSER

inherit

	PARAMETERS_FILE_PARSER

feature {ANY} -- Acces.

	parse_file (file_path: STRING): MAP_PARAMETERS
			-- Parse file with path `file_path'.
		local
			block_height, block_width: INTEGER_32
			inflation: REAL_64
			connectivity: GRID_CONNECTIVITY_STRATEGY
			file: PLAIN_TEXT_FILE
			key: STRING
		do

			create file.make_open_read (file_path)
			from
				file.start
			until
				file.off
			loop
				file.read_word
				create key.make_from_string (file.last_string)

				if key.is_equal ("block_height:") then
					file.read_integer
					block_height := file.last_integer
				elseif key.is_equal ("block_width:") then
					file.read_integer
					block_width := file.last_integer
				elseif key.is_equal ("inflation:") then
					file.read_double
					inflation := file.last_double
				elseif key.is_equal ("connectivity:") then
					file.read_integer
					inspect file.last_integer
					when 4 then
						connectivity := create {MANHATTAN_CONNECTIVITY_STRATEGY}
					when 8 then
						connectivity := create {FULL_CONNECTIVITY_STRATEGY}
					else
						io.putstring ("Parser error while parsing file '" + file_path + "': Value '" + file.last_integer.out + "' not recognized%N")
					end
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := create {MAP_PARAMETERS}.make_with_attributes (block_width, block_height, inflation, connectivity)
		end

end
