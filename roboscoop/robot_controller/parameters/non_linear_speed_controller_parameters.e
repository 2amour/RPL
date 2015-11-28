note
	description: "Non-linear speed controller parameters."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	NON_LINEAR_SPEED_CONTROLLER_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty NON_LINEAR_SPEED_CONTROLLER parameters object.
		do
		end

	make_with_attributes (nlsc_maximum_speed: REAL_64; nlsc_angular_decay_rate: REAL_64)
			-- Create NON_LINEAR_SPEED_CONTROLLER parameters object with attributes.
		do
			maximum_speed := nlsc_maximum_speed
			angular_decay_rate := nlsc_angular_decay_rate
		end

	make_from_separate (other: separate NON_LINEAR_SPEED_CONTROLLER_PARAMETERS)
			-- Create NON_LINEAR_SPEED_CONTROLLER parameters object from separate other.
		do
			maximum_speed := other.maximum_speed
			angular_decay_rate := other.angular_decay_rate
		end

feature -- Access

	maximum_speed: REAL_64
			-- NON_LINEAR_SPEED_CONTROLLER maximum_speed.

	angular_decay_rate: REAL_64
			-- NON_LINEAR_SPEED_CONTROLLER angular decay rate.

	set_maximum_speed (nlsc_maximum_speed: REAL_64)
			-- Setter for `maximum_speed'.
		do
			maximum_speed := nlsc_maximum_speed
		end

	set_angular_decay_rate (nlsc_angular_decay_rate: REAL_64)
			-- Setter for `angular_decay_rate'.
		do
			angular_decay_rate := nlsc_angular_decay_rate
		end
end
