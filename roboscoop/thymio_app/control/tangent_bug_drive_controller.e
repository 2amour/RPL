note
	description: "Summary description for {TANGENT_BUG_DRIVE_CONTROLLER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_DRIVE_CONTROLLER
inherit
	CANCELLABLE_CONTROL_LOOP

create make_with_attributes

feature -- Initialization	
	make_with_attributes (stop_sig: separate STOP_SIGNALER)
		-- Create controller given the attributes.
		do
			stop_signaler := stop_sig
		end

feature {TANGENT_BUG_BEHAVIOUR}
	update_velocity(tangent_bug_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP;
					g_sig: separate LIFTABLE; s_sig: separate STOP_SIGNALER;
					drive: separate DIFFERENTIAL_DRIVE)
		do
			if s_sig.is_stop_requested then
				drive.stop
			else

				tangent_bug_sig.get_pose.get_position.set_coordinates (o_sig.x, o_sig.y) -- TODO expanded class POSE?
				tangent_bug_sig.get_pose.set_orientation (o_sig.y) -- TODO
				tangent_bug_sig.set_timestamp (o_sig.timestamp)



				tangent_bug_sig.get_state.set_readings(tangent_bug_sig, r_sig)
				tangent_bug_sig.get_state.update_velocity (drive)
				tangent_bug_sig.get_state.update_state(tangent_bug_sig, o_sig, r_sig)

			end

		end



end
