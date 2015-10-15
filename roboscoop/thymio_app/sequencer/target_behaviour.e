note
	description: "Behavior for the robot to move towards a target."
	author: "Ferran Pallarès"
	date: "05.09.2015"

class
	TARGET_BEHAVIOUR

inherit
	BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (odom_sig: separate ODOMETRY_SIGNALER; d_drive: separate DIFFERENTIAL_DRIVE;
							but_leds: separate THYMIO_BUTTONS_LEDS; cir_leds: separate THYMIO_CIRCLE_LEDS; top_leds: separate THYMIO_TOP_LEDS)
			-- Create behaviour with given attributes.
		do
			create stop_signaler.make
			create target_signaler

			odometry_sig := odom_sig
			diff_drive := d_drive
			button_leds := but_leds
			circle_leds := cir_leds
			top_face_leds := top_leds
		end

feature -- Access

	set_target (x: REAL_64; y: REAL_64)
		do
			x_target := x
			y_target := y
		end

	start
			-- Start the behaviour.
		local
			constants: separate TARGET_CONSTANTS
			a, b: separate TARGET_CONTROLLER
			c: separate TARGET_LED_CONTROLLER
		do
			create constants.make

			a := create_target_controller (stop_signaler, constants)
			b := create_target_controller (stop_signaler, constants)
			create c.make (stop_signaler)

			sep_stop (stop_signaler, False)
			sep_start (a, b, c)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	target_signaler: separate TARGET_SIGNALER
			-- Signaler for controlling the behaviour state.

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

	x_target: REAL_64
			-- Target x coordinate.

	y_target: REAL_64
			-- Target y coordinate.

	sep_start (a, b: separate TARGET_CONTROLLER; c: separate TARGET_LED_CONTROLLER)
			-- Start behaviour controllers.
		do
			a.repeat_until_stop_requested (
				agent a.move_to_target (target_signaler, x_target, y_target, odometry_sig, diff_drive))

			b.repeat_until_stop_requested (
				agent b.stop_when_reached (target_signaler, x_target, y_target, odometry_sig, diff_drive))

			c.repeat_until_stop_requested (
				agent c.set_leds (target_signaler, button_leds, circle_leds, top_face_leds))
		end

	sep_stop (stop_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for move_toa stop.
		do
			stop_sig.set_stop_requested (val)
		end

	create_target_controller (stop_sig: separate STOP_SIGNALER; const: separate TARGET_CONSTANTS): TARGET_CONTROLLER
			-- Creates a new instance of TARGET_CONTROLLER.
		local
			a: TARGET_CONTROLLER
		do
			create a.make_with_attributes (stop_sig, const.kp, const.ki, const.kd, const.target_threshold, const.min_time_increment)
			Result := a
		end
end
