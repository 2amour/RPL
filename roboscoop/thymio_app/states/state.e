note
	description: "Summary description for {STATE}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

deferred class
	STATE

feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
		-- Update robot velocity
	deferred
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		-- Update LEDs color
	deferred
	end

	set_readings(o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		-- Set and process readings from sensor inputs
	deferred
	end

	update_state(t_sig: TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		-- update the signaler to the next state
	deferred
	end

end
