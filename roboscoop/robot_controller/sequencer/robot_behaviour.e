note
	description: "Behaviour of a robot."
	author: "Ferran Pallarès"
	date: "21.10.15"

deferred class
	ROBOT_BEHAVIOUR

inherit
	BEHAVIOUR

feature -- Access

	set_robot_parts (odom_sig: separate ODOMETRY_SIGNALER; r_group: separate RANGE_GROUP; g_group: separate LIFTABLE; d_drive: separate DIFFERENTIAL_DRIVE)
			-- Setter for the robot parts the behaviour may use.
		do
			odometry_sig := odom_sig
			range_group := r_group
			ground_group := g_group
			diff_drive := d_drive
		end

	start
			-- Start the behaviour.
		deferred
		end

	stop
			-- Stop the behaviour.
		deferred
		end

feature {NONE} -- Implementation

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	odometry_sig: detachable separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	range_group: detachable separate RANGE_GROUP
			-- Horizontal range sensors.

	ground_group: detachable separate LIFTABLE
			-- Ground sensors.

	diff_drive: detachable separate DIFFERENTIAL_DRIVE
			-- Object to control robot's speed.
end
