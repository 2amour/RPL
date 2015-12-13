note
	description: "Robot LED controller."
	author: "Sebastian Curi"
	date: "12.12.2015"

class
	LED_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (stop_sig: separate STOP_SIGNALER)
			-- Make `Current' with given attributes
		do
			stop_signaler := stop_sig
		end

feature -- Access

	update_leds (led_sig: separate LED_CONTROLLER_SIGNALER; cir_leds: separate LED_ARRAY; marker_sig: separate MARKER_SIGNALER; stop_sig: separate STOP_SIGNALER)
			-- Update leds module
		require
			marker_sig.is_new_val
		do
			marker_sig.set_new_val (False)
			if not stop_sig.is_stop_requested and
				led_sig.is_code (create {ARRAY[DOUBLE]}.make_from_array (<<marker_sig.data.color.r,
																		   marker_sig.data.color.g,
																		   marker_sig.data.color.b>>))
			then
				led_sig.increment_leds (cir_leds.number_of_leds)
				cir_leds.set_leds_brightness (get_array(led_sig.led_counter, cir_leds.number_of_leds))
			end
		end

feature {NONE} --Implementation

	get_array (a_number, a_size: INTEGER_32): ARRAY[INTEGER_16]
			-- Get array of leds to turn on
		local
			array: ARRAY[INTEGER_16]
			i: INTEGER
		do
			create array.make_filled (0, 1, a_size)

			from i:= 1
			until i > a_number
			loop
				array[i] := {LED_ARRAY}.led_on_value
				i := i + 1
			end
			Result := array
		end


end
