note
	description: "Class for speed control"
	author: "Sebastian Curi"
	date: "18.10.15"

class
	NON_LINEAR_SPEED_CONTROLLER

create
	make_with_attributes

feature -- Initialization

	make_with_attributes (nlsc_maximum_speed, nlsc_angular_decay_rate: REAL_64)
			-- Initialize object with max speed.
		do
			max_speed := nlsc_maximum_speed
			angular_decay_rate := nlsc_angular_decay_rate
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
			output := max_speed / (1 + angular_decay_rate*({DOUBLE_MATH}.log (1 + {DOUBLE_MATH}.dabs (w))))
		end

	get_output: REAL_64
			-- Output speed.
		do
			Result := output
		end

feature {NONE} -- Implementation

	max_speed: REAL_64
			-- Maximal speed.

	angular_decay_rate: REAL_64
			-- Angular decay rate.

	output: REAL_64
			-- Output speed.
end
