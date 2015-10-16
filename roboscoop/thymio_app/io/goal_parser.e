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
	make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		local
			x_goal: REAL_64
			y_goal: REAL_64
		do
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
		end

feature {NONE} --Initialization
	file_path: STRING = "goal.txt"
		-- File path in system.
feature -- Access
	goal: POINT_2D
end
