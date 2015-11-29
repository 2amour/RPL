note
	description: "Controls robot during tangent bug behaviour."
	author: "Ferran Pallarès"
	date: "06.11.15"

class
	TANGENT_BUG_CONTROLLER

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

	update_velocity (tangent_bug_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP;
					g_sig: separate LIFTABLE; s_sig: separate STOP_SIGNALER;
					drive: separate DIFFERENTIAL_DRIVE)
			-- Update velocity settings.
		do
			if s_sig.is_stop_requested then
				drive.stop
			else
				tangent_bug_sig.set_pose (create {separate POSE_2D}.make_with_coordinates (o_sig.x, o_sig.y, o_sig.theta))
				tangent_bug_sig.set_timestamp (o_sig.timestamp)

				tangent_bug_sig.state.set_readings (tangent_bug_sig, r_sig)
				tangent_bug_sig.state.update_velocity (drive)
				tangent_bug_sig.state.update_state (tangent_bug_sig, o_sig, r_sig)
			end
		end
end
