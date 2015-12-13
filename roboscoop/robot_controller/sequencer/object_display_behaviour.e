note
	description: "Behaviour to sequence the led display behaviour."
	author: "Sebastian Curi"
	date: "12/12/2015"

class
	OBJECT_DISPLAY_BEHAVIOUR

inherit
	BEHAVIOUR
create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (ros_topics_params: separate ROBOT_CONTROLLER_TOPIC_PARAMETERS)
			-- Create behaviour with given attributes.
		do
			create stop_signaler.make
			create circular_leds.make_with_topic (ros_topics_params.circular_leds_topic)
			create marker_signaler.make_with_topic (ros_topics_params.visualization_marker)
			create led_signaler.make_with_attributes (<<0.0, 1.0, 0.0>>) -- TODO-REMOVE HARD CODING
		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate LED_CONTROLLER
		do
			create a.make_with_attributes (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	led_signaler: separate LED_CONTROLLER_SIGNALER
			-- Signaler to manage object recognition.

	marker_signaler: separate MARKER_SIGNALER
			-- Signaler to listen marker msgs.

	circular_leds: separate THYMIO_CIRCLE_LEDS
			-- Thymio circle leds

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start (a: separate LED_CONTROLLER)
			-- Start behaviour controllers.
		do
			a.repeat_until_stop_requested (
					agent a.update_leds (led_signaler, circular_leds, marker_signaler, stop_signaler))
		end

	sep_stop (stop_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Manage stop signaler.
		do
			stop_sig.set_stop_requested (val)
		end

end
