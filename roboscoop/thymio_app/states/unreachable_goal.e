note
	description: "Summary description for {UNREACHABLE_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	UNREACHABLE_GOAL
inherit
	STATE
feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
	do
		drive.stop
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
	do
		leds.set_to_magenta
	end

	set_readings(odometry_signaler:separate ODOMETRY_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
	do
	end

	update_state(t_sig: TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
	do
	end

end
