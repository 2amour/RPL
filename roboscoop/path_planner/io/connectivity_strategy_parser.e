note
	description: "This class parses which connectivity strategy to use."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	CONNECTIVITY_STRATEGY_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			connectivity := create {MANHATTAN_CONNECTIVITY_STRATEGY}
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get connectivity strategy.
		do
			opened_file.read_integer
			inspect opened_file.last_integer
			when 4 then
				connectivity := create {MANHATTAN_CONNECTIVITY_STRATEGY}
			when 8 then
				connectivity := create {FULL_CONNECTIVITY_STRATEGY}
			else
				io.putstring (opened_file.last_integer.out + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
		end

feature {ANY} -- Access

	connectivity: GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity strategy

end
