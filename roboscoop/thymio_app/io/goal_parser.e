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
	make_with_path

feature {NONE} -- Initialization

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		local
			x_goal: REAL_64
			y_goal: REAL_64
		do
			file_path := path
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

feature -- Access
	goal: POINT_2D
end
