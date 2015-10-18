note
	description: "Controls robot speed during tangent bug behaviour."
	author: "ferran_antoni_sebastian"
	date: "18.10.15"

class
	TANGENT_BUG_DRIVE_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make_with_attributes

feature {NONE} -- Initialization	

	make_with_attributes (stop_sig: separate STOP_SIGNALER)
		-- Create controller given the attributes.
		do
			stop_signaler := stop_sig
		end

feature {TANGENT_BUG_BEHAVIOUR} -- Access

	update_velocity(tangent_bug_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP;
					g_sig: separate LIFTABLE; s_sig: separate STOP_SIGNALER;
					drive: separate DIFFERENTIAL_DRIVE)
					-- Update velocity settings.
		do
			if s_sig.is_stop_requested then
				drive.stop
			else

				tangent_bug_sig.get_pose.get_position.set_coordinates (o_sig.x, o_sig.y) -- TODO expanded class POSE?
				tangent_bug_sig.get_pose.set_orientation (o_sig.theta)
				tangent_bug_sig.set_timestamp (o_sig.timestamp)

				tangent_bug_sig.get_state.set_readings(tangent_bug_sig, r_sig)
				tangent_bug_sig.get_state.update_velocity (drive)
				tangent_bug_sig.get_state.update_state(tangent_bug_sig, o_sig, r_sig)
			end
		end
end
