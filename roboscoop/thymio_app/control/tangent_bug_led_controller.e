note
	description: "Controls LEDs during tangent bug behaviour."
	author: "ferran_antoni_sebastian"
	date: "18.10.2015"

class
	TANGENT_BUG_LED_CONTROLLER

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

	update_leds (tangent_bug_sig: separate TANGENT_BUG_SIGNALER; s_sig: separate STOP_SIGNALER; leds: separate RGB_COLOR_ACTUATOR)
			-- Update LEDs setting.
		do
			if s_sig.is_stop_requested then
				leds.set_to_white
			else
				tangent_bug_sig.get_state.update_leds (leds)
			end
		end
end
