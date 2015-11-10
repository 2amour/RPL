note
	description: "Summary description for {TANGENT_BUG_STATE}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

deferred class
	TANGENT_BUG_STATE

feature

	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
		-- Update robot velocity.
	deferred
	end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; r_sig: separate RANGE_GROUP)
		-- Set and process readings from sensor inputs.
	deferred
	end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
		-- update the signaler to the next state.
	deferred
	end

end
