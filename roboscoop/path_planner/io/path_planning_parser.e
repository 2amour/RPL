note
	description: "Main parser for this application."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	PATH_PLANNING_PARSER

inherit

	CMD_LINE_ARGS_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			create way_points_parser.make
			create connectivity_parser.make
			create edge_cost_parser.make
			create heuristic_cost_parser.make
			create open_set_strategy_parser.make
			create map_inflation_parser.make
		end

feature {NONE} -- Implementation.

	parse (idx: INTEGER_32; arg_val: STRING_8)
			-- Parse each argument
		local
			paths: ARRAYED_QUEUE [STRING]
			file_path_parser: FILE_PATHS_PARSER
		do
			create paths.make (0)
			create file_path_parser.make
			file_path_parser.parse_path (arg_val)
			paths := file_path_parser.file_paths
			way_points_parser.parse_path (paths.item)
			paths.remove
			connectivity_parser.parse_path (paths.item)
			paths.remove
			edge_cost_parser.parse_path (paths.item)
			paths.remove
			heuristic_cost_parser.parse_path (paths.item)
			paths.remove
			open_set_strategy_parser.parse_path (paths.item)
			paths.remove
			map_inflation_parser.parse_path (paths.item)
			paths.remove
		end

feature {ANY} --Access

	way_points_parser: WAY_POINT_PARSER
			-- Way point parser.

	connectivity_parser: CONNECTIVITY_STRATEGY_PARSER
			-- Grid connectivity strategy parser

	edge_cost_parser: HEURISTIC_COST_PARSER
			-- Edge cost function parser.

	heuristic_cost_parser: HEURISTIC_COST_PARSER
			-- Heuristic cost function parser.

	open_set_strategy_parser: OPEN_SET_STRATEGY_PARSER
			-- Open set structure parser.

	map_inflation_parser: MAP_INFLATION_PARSER
			-- Map parameter parser.

end
