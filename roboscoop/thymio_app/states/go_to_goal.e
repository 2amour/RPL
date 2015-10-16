note
	description: "Summary description for {GO_TO_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	GO_TO_GOAL

inherit
	STATE
feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
	do
		drive.stop
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
	do
		leds.set_to_yellow
	end

	set_readings(odometry_signaler:separate ODOMETRY_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
	do
	end

	update_state(t_sig: TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
	local
		new_state: TANGENT_BUG_STATE
	do
		if r_sig.is_obstacle_in_front then
			create new_state.make_with_state ({TANGENT_BUG_STATE}.follow_wall)
			t_sig.set_state (new_state)
		end
	end

end
