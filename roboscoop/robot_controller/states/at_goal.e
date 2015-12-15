note
	description: "In this state the robot has already reached the goal."
	author: "Ferran Pallarès"
	date: "28.11.2015"

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

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		do
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if t_sig.goal.get_position.get_euclidean_distance (t_sig.current_pose.get_position) > t_sig.reached_pose_position_threshold or
				{DOUBLE_MATH}.dabs (t_sig.goal.get_orientation - t_sig.current_pose.get_orientation) > t_sig.reached_pose_orientation_threshold then
				t_sig.set_go_to_goal
			end
		end
end
