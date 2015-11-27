note
	description: "Summary description for {AT_GOAL}."
	author: ""
	date: "$Date$"


class
	AT_GOAL

inherit
	TANGENT_BUG_STATE

feature -- Access
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			drive.stop
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
			-- <Precursor>
		do
			leds.set_to_green
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		do
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if t_sig.goal.get_euclidean_distance (t_sig.current_pose.get_position) > t_sig.goal_threshold then
				t_sig.set_go_to_goal
			end
		end
end
