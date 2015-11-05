note
	description: "This class parses the open_set_strategy from a txt file."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	OPEN_SET_STRATEGY_PARSER

inherit

	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			make_default
			bfs := True
			dfs := False
			dijkstra := False
		end

feature {NONE} -- Implementation

	parse (opened_file: PLAIN_TEXT_FILE)
			-- Parse file to get graph search paameters.
		do
			opened_file.read_word_thread_aware
			if opened_file.last_string.is_equal ("bfs") then
				bfs := True
				dfs := False
				dijkstra := False
			elseif opened_file.last_string.is_equal ("dfs") then
				bfs := False
				dfs := True
				dijkstra := False
			elseif opened_file.last_string.is_equal ("dijkstra") then
				bfs := False
				dfs := False
				dijkstra := True
			else
				io.putstring (opened_file.last_string + " in file " + file_path + " not recognized for parsing.%N")
				exception.raise ("Parsing Error")
			end
			debug
				io.put_string ("Parsing opened set %N")
			end
		end

feature {ANY} -- Access

	bfs: BOOLEAN
			-- FIFO queue.

	dfs: BOOLEAN
			-- LIFO stack.

	dijkstra: BOOLEAN
			-- Heap Stack

end
