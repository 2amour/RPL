note
	description: "This class parses parameters for the path-planning algorithm."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS_PARSER

inherit

	PARAMETERS_FILE_PARSER


feature {ANY} -- Acces.

	parse_file (file_path: separate STRING): PATH_PLANNER_PARAMETERS
			-- Parse file with path `file_path'.
		local
			parameters: PATH_PLANNER_PARAMETERS
			edge_key, heuristic_key, search_key: STRING
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			-- Default init.
			create parameters.make_default
			edge_key := "" heuristic_key := "" search_key:= ""

			create file.make_open_read (create {STRING}.make_from_separate (file_path))
			from
				file.start
			until
				file.off
			loop
				file.read_word
				create key.make_from_string (file.last_string)
				if key.is_equal ("frame_id:") then
					file.read_word
					parameters.set_frame_id (file.last_string)
				elseif key.is_equal ("edge_cost:") then
					file.read_word
					create edge_key.make_from_string (file.last_string)
					if edge_key.is_equal ("euclidean") then
						parameters.set_edge_cost (create {EUCLIDEAN_HEURISTIC})
					elseif edge_key.is_equal ("infinity") then
						parameters.set_edge_cost (create {INFINITY_NORM_HEURISTIC})
					elseif edge_key.is_equal ("manhattan") then
						parameters.set_edge_cost (create {MANHATTAN_HEURISTIC})
					elseif edge_key.is_equal ("triangle") then
						parameters.set_edge_cost (create {TRIANGLE_HEURISTIC})
					elseif edge_key.is_equal ("zero") then
						parameters.set_edge_cost (create {ZERO_HEURISTIC})
					elseif not edge_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + edge_key + "' not recognized%N")
					end
				elseif key.is_equal ("heuristic:") then
					file.read_word
					create heuristic_key.make_from_string (file.last_string)
					if heuristic_key.is_equal ("euclidean") then
						parameters.set_heuristic_cost (create {EUCLIDEAN_HEURISTIC})
					elseif heuristic_key.is_equal ("infinity") then
						parameters.set_heuristic_cost (create {INFINITY_NORM_HEURISTIC})
					elseif heuristic_key.is_equal ("manhattan") then
						parameters.set_heuristic_cost (create {MANHATTAN_HEURISTIC})
					elseif heuristic_key.is_equal ("triangle") then
						parameters.set_heuristic_cost (create {TRIANGLE_HEURISTIC})
					elseif heuristic_key.is_equal ("zero") then
						parameters.set_heuristic_cost (create {ZERO_HEURISTIC})
					elseif not heuristic_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + heuristic_key + "' not recognized%N")
					end
				elseif key.is_equal ("search_strategy:") then
					file.read_word
					create search_key.make_from_string (file.last_string)
					if search_key.is_equal ("dijkstra") then
						parameters.set_search_strategy (create {A_STAR}.make_default)
					elseif heuristic_key.is_equal ("bfs") then
						parameters.set_search_strategy (create {BFS}.make_default)
					elseif heuristic_key.is_equal ("dfs") then
						parameters.set_search_strategy (create {DFS}.make_default)
					elseif not heuristic_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + search_key + "' not recognized%N")
					end
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close
			Result := parameters
		end

end

