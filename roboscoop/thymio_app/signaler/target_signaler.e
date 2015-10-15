note
	description: "State of the TARGET_BEHAVIOUR."
	author: "Ferran Pallarès"
	date: "06.10.2015"

class
	TARGET_SIGNALER

feature -- Access

	is_target_reached: BOOLEAN
			-- Has the target been reached?

	set_target_reached (a_val: BOOLEAN)
			-- Set `target_reached' value equal to `a_val'.
		do
			is_target_reached := a_val
		end

end
