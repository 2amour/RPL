note
	description: "In this state the robot is aware that the goal is unreachable."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	UNREACHABLE_GOAL

inherit
	TANGENT_BUG_STATE

feature --Access

	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			drive.stop
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		do
			if unreachable_goal = void then
				unreachable_goal := t_sig.goal
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if not (unreachable_goal = t_sig.goal) then
				unreachable_goal := void
				t_sig.set_has_turned_back (False)
				t_sig.set_go_to_goal
			end
		end

feature {NONE} -- Implementation

	unreachable_goal: detachable separate POINT_2D
			-- Unreachable goal.

end
