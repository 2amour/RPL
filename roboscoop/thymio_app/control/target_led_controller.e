note
	description: "Robot LED controller for TARGET_BEHAVIOUR"
	author: "Ferran Pallarès"
	date: "05.10.2015"

class
	TARGET_LED_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make

feature {NONE} -- Initialization

	make (stop_sig: separate STOP_SIGNALER)
		do
			stop_signaler := stop_sig
		end

feature -- Constants

	blink_period: INTEGER_64 = 250
		-- Minimum lapse of time (in msec) between state checks.

feature -- Access

	prev_time: INTEGER_64
			-- Last time (in msec) the algorithm was run.

	set_leds (pid_sig: separate TARGET_SIGNALER; but_leds: separate THYMIO_BUTTONS_LEDS; cir_leds: separate THYMIO_CIRCLE_LEDS; top_leds: separate THYMIO_TOP_LEDS)
			-- Set robot's LEDs depending on situation.
		local
			current_time: INTEGER_64
		do
			current_time := {TIMER}.current_time_millis

			if sep_is_stop_requested (stop_signaler) then
				top_leds.set_to_red
				but_leds.set_leds_brightness (<<32, 32, 32, 32>>)
				cir_leds.set_leds_brightness (<<32, 32, 32, 32, 32, 32, 32, 32>>)

			elseif prev_time = 0 then
				prev_time := current_time

			elseif (current_time - prev_time) >= blink_period then
				if pid_sig.is_target_reached then
					top_leds.set_to_green
					if blink then
						but_leds.set_leds_brightness (<<32, 0, 32, 0>>)
						cir_leds.set_leds_brightness (<<32, 0, 32, 0, 32, 0, 32, 0>>)
					else
						but_leds.set_leds_brightness (<<0, 32, 0, 32>>)
						cir_leds.set_leds_brightness (<<0, 32, 0, 32, 0, 32, 0, 32>>)
					end
					blink := not blink
				else
					top_leds.set_to_yellow
					but_leds.set_leds_brightness (<<32, 0, 32, 0>>)
					cir_leds.set_leds_brightness (<<32, 0, 0, 0, 32, 0, 0, 0>>)
				end

				prev_time := current_time
			end
		end

feature {NONE} -- Implementation

	blink: BOOLEAN
			-- LED blink state when goal reached.
end
