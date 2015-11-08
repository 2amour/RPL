note
	description: "Tangent bug parameters bag that groups different types of parameters."
	author: "Ferran Pallarès"
	date: "22.10.2015"

class
	TANGENT_BUG_PARAMETERS_BAG

inherit
	PARAMETERS_BAG

create
	make_with_attributes

feature {NONE} -- Implementation

	make_with_attributes (tangent_bug_goal_parameters: GOAL_PARAMETERS; tangent_bug_pid_parameters: PID_PARAMETERS; tangent_bug_wall_following_parameteres: WALL_FOLLOWING_PARAMETERS; tangent_bug_range_sensors_parameters: RANGE_SENSORS_PARAMETERS)
			-- Make tangent bug parameters bag from attributes.
		do
			goal_parameters := tangent_bug_goal_parameters
			pid_parameters := tangent_bug_pid_parameters
			wall_following_parameters := tangent_bug_wall_following_parameteres
			range_sensors_parameters := tangent_bug_range_sensors_parameters
		end

feature -- Access


	goal_parameters: GOAL_PARAMETERS
			-- Tangent bug goal parameters.

	pid_parameters: PID_PARAMETERS
			-- Tangent bug PID parameters.

	wall_following_parameters: WALL_FOLLOWING_PARAMETERS
			-- Tangent bug wall following parameters.

	range_sensors_parameters: RANGE_SENSORS_PARAMETERS
			-- Tangent bug range sensors parameters.
end
