note
	description: "Main parser for this application."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	MISSION_PLANNER_PARSER

inherit

	CMD_LINE_ARGS_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialize `Current'.
		do
			create way_points_parser.make
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
		end

feature {ANY} --Access

	way_points_parser: WAY_POINT_PARSER
			-- Way point parser.

end
