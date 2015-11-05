note
	description: "This class parses which heuristic cost strategy to use."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	HEURISTIC_COST_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			cost := create {ZERO_HEURISTIC}
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get graph search paameters.
		do
			opened_file.read_word_thread_aware
			if opened_file.last_string.is_equal ("zero") then
				cost := create {ZERO_HEURISTIC}
			elseif opened_file.last_string.is_equal ("manhattan") then
				cost := create {MANHATTAN_HEURISTIC}
			elseif opened_file.last_string.is_equal ("euclidean") then
				cost := create {EUCLIDEAN_HEURISTIC}
			elseif opened_file.last_string.is_equal ("infinite") then
				cost := create {INFINITY_NORM_HEURISTIC}
			elseif opened_file.last_string.is_equal ("triangle") then
				cost := create {TRIANGLE_HEURISTIC}
			else
				io.putstring (opened_file.last_string + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
			debug
				io.put_string ("Parsing heuristic %N")
			end
		end

feature {ANY} -- Access

	cost: COST_HEURISTIC
			-- Heuristic class for a* algorithm.

end
