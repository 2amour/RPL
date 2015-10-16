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
		
end
