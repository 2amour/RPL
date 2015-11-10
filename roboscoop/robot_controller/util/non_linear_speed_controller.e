note
	description: "Class for speed control"
	author: "Sebastian Curi"
	date: "18.10.15"

class
	NON_LINEAR_SPEED_CONTROLLER

create
	make, make_with_speed

feature -- Initialization

	make
			-- Initialize object with default values.
		do
			max_speed := 0.08 -- Initialized by default
			output := 0.0
		end

	make_with_speed (v: REAL_64)
			-- Initialize object with max speed.
		do
			max_speed := v
			output := 0.0
		end

feature -- Access

	set_max_speed (v: REAL_64)
			-- Set maximum allowable speed.
		do
			max_speed := v
		ensure
			max_speed_set: max_speed = v
		end

	set_angular_velocity (w: REAL_64)
			-- Set Input to controller.
		do
			output := max_speed / (1 + {DOUBLE_MATH}.log ({DOUBLE_MATH}.dabs (w) + 2.0) )
		end

	get_output: REAL_64
			-- Output speed.
		do
			Result := output
		end

feature {NONE} -- Implementation

	max_speed: REAL_64
			-- Maximal speed.

	output: REAL_64
			-- Output speed.
end
