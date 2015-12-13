note
	description: "Deferred class to actuate on a led array."
	author: "Sebastian Curi"
	date: "12/12/2015"

deferred class
	LED_ARRAY

feature -- Access
	number_of_leds: INTEGER
			-- Number of leds this array.

	led_on_value: INTEGER_16 = 255
			-- Value that sets the led on.

	set_leds_brightness (a_leds: separate ARRAY[INTEGER_16])
			-- Set brightness for circle LEDs with the values from `a_leds'.
		deferred
		end
end
