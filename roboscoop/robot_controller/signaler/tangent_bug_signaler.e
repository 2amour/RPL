note
	description: "State of tangent bug behaviour."
	author: "Ferran Pallarès"
	date: "06.11.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (goal_parameters: separate GOAL_PARAMETERS; pid_parameters: separate PID_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Initialize signaler with attributes.
		do
		end
end
