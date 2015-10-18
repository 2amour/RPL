note
	description: "Summary description for {GOAL_PARSER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	GOAL_PARSER

inherit
	FILE_PARSER

create
	make_with_path, make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		local
			paths_parser: PATHS_PARSER
		do
			create paths_parser.make
			read_file (paths_parser.goal_file_path)
		end

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		do
			read_file (path)
		end


feature -- Access
	goal: POINT_2D

feature {NONE} -- Implementation

	read_file (f_path: STRING)
			-- Reads file.
		local
			x_goal: REAL_64
			y_goal: REAL_64
		do
			file_path := f_path
			create file.make_open_read (file_path)

			from file.start;
			until file.off
			loop
				file.read_character
				file.read_double
				inspect file.last_character
				when 'x' then
					x_goal := file.last_double
				when 'y' then
					y_goal := file.last_double
				else
					io.putstring ("Character not recognized")
				end
			end

			create goal.make_with_coordinates (x_goal, y_goal)
			file.close

			debug
				io.put_string ("x_goal: " + goal.get_x.out + ", y_goal: " + goal.get_y.out + "%N")
			end
		end
end
