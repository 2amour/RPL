note
	description: "Behaviour of a Thymio robot."
	author: "Ferran Pallarès"
	date: "21.10.15"

deferred class
	THYMIO_BEHAVIOUR

inherit
	BEHAVIOUR

feature {NONE} -- Initialization


	make_with_attributes (parameters_bag: separate PARAMETERS_BAG)
			-- Create behaviour with given attributes.
		deferred
		end

feature -- Access

	set_robot_parts (odom_sig: separate ODOMETRY_SIGNALER; r_group: separate THYMIO_RANGE_GROUP;
						g_group: separate LIFTABLE; d_drive: separate DIFFERENTIAL_DRIVE;
						but_leds: separate THYMIO_BUTTONS_LEDS; cir_leds: separate THYMIO_CIRCLE_LEDS; top_leds: separate THYMIO_TOP_LEDS)
			-- Setter for the robot parts the behaviour may use.
		do
			odometry_sig := odom_sig
			range_group := r_group
			ground_group := g_group
			diff_drive := d_drive
			button_leds := but_leds
			circle_leds := cir_leds
			top_face_leds := top_leds
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

	range_group: separate THYMIO_RANGE_GROUP
			-- Horizontal range sensors.

	ground_group: separate LIFTABLE
			-- Ground sensors.

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	tangent_bug_signaler: separate TANGENT_BUG_SIGNALER
			-- Signaler for controlling the tangent bug state.

	odometry_sig: separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	diff_drive: separate DIFFERENTIAL_DRIVE
			-- Object to control robot's speed.

	button_leds: separate THYMIO_BUTTONS_LEDS
			-- Robot button leds.

	circle_leds: separate THYMIO_CIRCLE_LEDS
			-- Robot circle leds.

	top_face_leds: separate THYMIO_TOP_LEDS
			-- Robot top-face leds.
end
