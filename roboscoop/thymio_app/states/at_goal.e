note
	description: "Summary description for {AT_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	AT_GOAL
inherit
	STATE
feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
		do
			drive.stop
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		do
			leds.set_to_green
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
		do
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		do
			if t_sig.get_goal.get_euclidean_distance (t_sig.get_pose.get_position) < 0.05 then -- TODO, REMOVE THIS HARDCODING
				t_sig.set_go_to_goal
			end
		end
end
