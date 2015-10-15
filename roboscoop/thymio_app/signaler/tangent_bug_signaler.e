note
	description: "State of Tangent bug."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_SIGNALER
	
create
	make

feature{NONE} -- Private methods
	--clear_all
		-- Clear All states
--	do
--		go_to_goal := False
--		follow_wall := False
--		leave_wall := False
--		at_goal := False
--		in_danger := False
--		unreachable_goal := False
--	end


feature --Initialization
	make
		-- Init to go_to_goal
	do
		set_go_to_goal
	end

feature --Accesors

	set_go_to_goal
		-- Set state
	do

	end

end
