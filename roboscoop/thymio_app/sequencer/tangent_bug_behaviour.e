note
	description: "Summary description for {TANGENT_BUG_BEHAVIOUR}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_BEHAVIOUR
inherit
	BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (odom_sig: separate ODOMETRY_SIGNALER; r_group: separate THYMIO_RANGE_GROUP;
							g_group: separate LIFTABLE; d_drive: separate DIFFERENTIAL_DRIVE;
							but_leds: separate THYMIO_BUTTONS_LEDS; cir_leds: separate THYMIO_CIRCLE_LEDS; top_leds: separate THYMIO_TOP_LEDS)
			-- Create behaviour with given attributes.
		local
			goal_parser: GOAL_PARSER
		do
			create stop_signaler.make
			create goal_parser.make_with_path ("goal.txt")
			create tangent_bug_signaler.make_with_goal (goal_parser.goal)

			odometry_sig := odom_sig
			range_group := r_group
			ground_group := g_group
			diff_drive := d_drive
			button_leds := but_leds
			circle_leds := cir_leds
			top_face_leds := top_leds
		end

feature -- Access
	start
			-- Start the behaviour.
		local
			a: separate TANGENT_BUG_DRIVE_CONTROLLER
			b: separate TANGENT_BUG_LED_CONTROLLER
		do
			create a.make_with_attributes(stop_signaler)
			create b.make_with_attributes(stop_signaler)
			sep_stop(stop_signaler, False)
			sep_start(a, b)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
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
			-- Robot top-face leds

	sep_start (a: separate TANGENT_BUG_DRIVE_CONTROLLER; b: separate TANGENT_BUG_LED_CONTROLLER)
			-- Start behaviour controllers.
		do
			a.repeat_until_stop_requested (
				agent a.update_velocity (tangent_bug_signaler, odometry_sig, range_group, ground_group, stop_signaler, diff_drive))

			b.repeat_until_stop_requested (
				agent b.update_leds (tangent_bug_signaler, stop_signaler, top_face_leds))

		end

	sep_stop (stop_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for move_toa stop.
		do
			stop_sig.set_stop_requested (val)
		end
end
