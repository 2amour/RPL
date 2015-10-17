note
	description: "Summary description for {NON_LINEAR_SPEED}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	NON_LINEAR_SPEED

create
	make, make_with_speed

feature{NONE} --Attributes
	max_speed: REAL_64
	output: REAL_64

feature -- Initialization
	make
			-- Initialize Object with Default Values
		do
			max_speed := {THYMIO_ROBOT}.default_linear_speed -- TODO change this
			output := 0.0
		end

	make_with_speed (v: REAL_64)
			-- Initialize object with max speed
		do
			max_speed := v
			output := 0.0
		end


feature --Accesors
	set_max_speed(v: REAL_64)
		-- Set maximum allowable speed
	do
		max_speed := v
	ensure
		max_speed_set: max_speed = v
	end


	set_angular_velocity(w: REAL_64)
		-- Set Input to controller
	do
		output := max_speed/(1 + {DOUBLE_MATH}.log({DOUBLE_MATH}.dabs(w)+2.0) )
	end


	get_output: REAL_64
	do
		Result := output
	end

end
