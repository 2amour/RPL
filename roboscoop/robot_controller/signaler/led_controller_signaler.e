note
	description: "LED controller signaler."
	author: "Sebastian Curi"
	date: "13/12/2015"

class
	LED_CONTROLLER_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_rgb_code: separate ARRAY[DOUBLE])
			-- Initialize `Current' with attributes.	
		local
			i: INTEGER
		do
			create rgb_code.make_filled (0, 1, a_rgb_code.count)
			from
				i := 1
			until
				i > a_rgb_code.count
			loop
				rgb_code.force (a_rgb_code[i], i)
				i := i + 1
			end

			led_counter := 0
		end

feature {NONE} -- Implementaiton

	rgb_code: ARRAY[DOUBLE]
			-- Code to check

feature --Access

	led_counter: INTEGER
			-- Number of leds that are on.

	increment_leds (leds_in_array: INTEGER)
			-- Increment led_counter.
		do
			led_counter := led_counter\\leds_in_array + 1
		end

	reset_led_counter
			-- Reset to 0 the led_counter.
		do
			led_counter := 0
		end

	is_code (rgb_test: separate ARRAY[DOUBLE]): BOOLEAN
			-- Check if code is valid
		require
			rgb_test.count = 3
		local
			i: INTEGER
		do
			Result := True
			from
				i := 1
			until
				i > rgb_code.count
			loop
				Result := Result and {DOUBLE_MATH}.dabs (rgb_test[i] - rgb_code[i]) < 2.0*{DOUBLE}.epsilon
				i := i + 1
			end
		end
end
