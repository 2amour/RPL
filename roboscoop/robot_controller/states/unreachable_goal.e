note
    description: "In this state the robot is aware that the goal is unreachable."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	UNREACHABLE_GOAL

inherit
	TANGENT_BUG_STATE

feature

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
            -- TODO - What happens when the goal is updated?
        end
end
