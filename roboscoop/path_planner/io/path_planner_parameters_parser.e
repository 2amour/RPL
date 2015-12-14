note
	description: "This class parses parameters for the path-planning algorithm."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make_default
		end

feature {ANY} -- Access

	parse_file (file_path: separate STRING)
			-- Parse file with path `file_path'.
		local
			edge_key, heuristic_key, search_key: STRING
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			edge_key := "" heuristic_key := "" search_key:= ""
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				file.open_read
				from
					file.start
				until
					file.off
				loop
					file.read_word_thread_aware
					key := file.last_string.twin
					if key.is_equal ("frame_id:") then
						file.read_word_thread_aware
						last_parameters.set_frame_id (file.last_string.twin)
					elseif key.is_equal ("edge_cost:") then
						file.read_word_thread_aware
						edge_key := file.last_string.twin
						if edge_key.is_equal ("euclidean") then
							last_parameters.set_edge_cost (create {EUCLIDEAN_HEURISTIC})
						elseif edge_key.is_equal ("infinity") then
							last_parameters.set_edge_cost (create {INFINITY_NORM_HEURISTIC})
						elseif edge_key.is_equal ("manhattan") then
							last_parameters.set_edge_cost (create {MANHATTAN_HEURISTIC})
						elseif edge_key.is_equal ("triangle") then
							last_parameters.set_edge_cost (create {TRIANGLE_HEURISTIC})
						elseif edge_key.is_equal ("zero") then
							last_parameters.set_edge_cost (create {ZERO_HEURISTIC})
						elseif not edge_key.is_empty then
							io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + edge_key + "' not recognized%N")
							is_error_found := True
						end
					elseif key.is_equal ("heuristic:") then
						file.read_word_thread_aware
						heuristic_key := file.last_string.twin
						if heuristic_key.is_equal ("euclidean") then
							last_parameters.set_heuristic_cost (create {EUCLIDEAN_HEURISTIC})
						elseif heuristic_key.is_equal ("infinity") then
							last_parameters.set_heuristic_cost (create {INFINITY_NORM_HEURISTIC})
						elseif heuristic_key.is_equal ("manhattan") then
							last_parameters.set_heuristic_cost (create {MANHATTAN_HEURISTIC})
						elseif heuristic_key.is_equal ("triangle") then
							last_parameters.set_heuristic_cost (create {TRIANGLE_HEURISTIC})
						elseif heuristic_key.is_equal ("zero") then
							last_parameters.set_heuristic_cost (create {ZERO_HEURISTIC})
						elseif not heuristic_key.is_empty then
							io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + heuristic_key + "' not recognized%N")
							is_error_found := True
						end
					elseif key.is_equal ("search_strategy:") then
						file.read_word_thread_aware
						search_key := file.last_string.twin
						if search_key.is_equal ("dijkstra") then
							last_parameters.set_search_strategy (create {A_STAR}.make_default)
						elseif heuristic_key.is_equal ("bfs") then
							last_parameters.set_search_strategy (create {BFS}.make_default)
						elseif heuristic_key.is_equal ("dfs") then
							last_parameters.set_search_strategy (create {DFS}.make_default)
						elseif not heuristic_key.is_empty then
							io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + search_key + "' not recognized%N")
							is_error_found := True
						end
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
			else
				is_error_found := True
			end
		end

		last_parameters: PATH_PLANNER_PARAMETERS
end

