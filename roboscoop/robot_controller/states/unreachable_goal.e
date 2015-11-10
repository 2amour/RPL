note
	description: "Summary description for {UNREACHABLE_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	UNREACHABLE_GOAL
inherit
	TANGENT_BUG_STATE
feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
	do
		drive.stop
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
	do
		leds.set_to_magenta
	end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
	do
	end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
	do
	end

end
