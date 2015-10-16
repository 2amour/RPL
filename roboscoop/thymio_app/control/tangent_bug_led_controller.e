note
	description: "Summary description for {TANGENT_BUG_LED_CONTROLLER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TANGENT_BUG_LED_CONTROLLER

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
	update_leds (tangent_bug_sig: separate TANGENT_BUG_SIGNALER; s_sig: separate STOP_SIGNALER; leds: separate RGB_COLOR_ACTUATOR)
		do
			tangent_bug_sig.get_state.update_leds (leds)

		end
end
