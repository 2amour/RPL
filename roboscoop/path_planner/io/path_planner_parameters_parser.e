note
	description: "This class parses parameters for the path-planning algorithm."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS_PARSER

inherit

	PARAMETERS_FILE_PARSER


feature {ANY} -- Acces.

	parse_file (file_path: STRING): PATH_PLANNER_PARAMETERS
			-- Parse file with path `file_path'.
		local
			frame_id: STRING_8
			edge_cost, heuristic_cost: COST_HEURISTIC
			bfs, dfs, dijkstra: BOOLEAN
			edge_key, heuristic_key, search_key: STRING
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

				if key.is_equal ("frame_id:") then
					file.read_word
					create frame_id.make_from_string (file.last_string)
				elseif key.is_equal ("edge_cost:") then
					file.read_word
					create edge_key.make_from_string (file.last_string)
					if edge_key.is_equal ("euclidean") then
						edge_cost := create {EUCLIDEAN_HEURISTIC}
					elseif edge_key.is_equal ("infinity") then
						edge_cost := create {INFINITY_NORM_HEURISTIC}
					elseif edge_key.is_equal ("manhattan") then
						edge_cost := create {MANHATTAN_HEURISTIC}
					elseif edge_key.is_equal ("triangle") then
						edge_cost := create {TRIANGLE_HEURISTIC}
					elseif edge_key.is_equal ("zero") then
						edge_cost := create {ZERO_HEURISTIC}
					elseif not edge_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + edge_key + "' not recognized%N")
					end
				elseif key.is_equal ("heuristic:") then
					file.read_word
					create heuristic_key.make_from_string (file.last_string)
					if heuristic_key.is_equal ("euclidean") then
						heuristic_cost := create {EUCLIDEAN_HEURISTIC}
					elseif heuristic_key.is_equal ("infinity") then
						heuristic_cost := create {INFINITY_NORM_HEURISTIC}
					elseif heuristic_key.is_equal ("manhattan") then
						heuristic_cost := create {MANHATTAN_HEURISTIC}
					elseif heuristic_key.is_equal ("triangle") then
						heuristic_cost := create {TRIANGLE_HEURISTIC}
					elseif heuristic_key.is_equal ("zero") then
						heuristic_cost := create {ZERO_HEURISTIC}
					elseif not heuristic_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + heuristic_key + "' not recognized%N")
					end
				elseif key.is_equal ("search_strategy:") then
					file.read_word
					create search_key.make_from_string (file.last_string)
					if search_key.is_equal ("dijkstra") then
						bfs := False dfs := False dijkstra := True
					elseif heuristic_key.is_equal ("bfs") then
						bfs := True dfs := False dijkstra := False
					elseif heuristic_key.is_equal ("dfs") then
						bfs := False dfs := True dijkstra := False
					elseif not heuristic_key.is_empty then
						io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + search_key + "' not recognized%N")
					end
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := create {PATH_PLANNER_PARAMETERS}.make_with_attributes (frame_id, edge_cost, heuristic_cost, bfs, dfs, dijkstra)
		end

end

