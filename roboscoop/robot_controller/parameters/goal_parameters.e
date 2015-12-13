note
	description: "Goal parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	GOAL_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty goal parameters object.
		do
		end

	make_with_attributes (goal_x: REAL_64; goal_y: REAL_64; goal_orientation: REAL_64; goal_threshold: REAL_64)
			-- Create goal parameters object with attributes.
		do
			x := goal_x
			y := goal_y
			orientation := goal_orientation
			threshold := goal_threshold
		end

	make_from_separate (other: separate GOAL_PARAMETERS)
			-- Create goal parameters object from separate other.
		do
			x := other.x
			y := other.y
			orientation := other.orientation
			threshold := other.threshold
		end

feature -- Access

	x: REAL_64
			-- Goal x coordinate.

	y: REAL_64
			-- Goal y coordinate.

	orientation: REAL_64
			-- Goal orientation.

	threshold: REAL_64
			-- Threshold for considering when the goal is reached.

	set_x (goal_x: REAL_64)
			-- Setter for `x'.
		do
			x := goal_x
		end

	set_y (goal_y: REAL_64)
			-- Setter for `y'.
		do
			y := goal_y
		end

	set_orientation (goal_orientation: REAL_64)
			-- Setter for `orientation'.
		do
			orientation := goal_orientation
		end

	set_threshold (goal_threshold: REAL_64)
			-- Setter for `threshold'.
		do
			threshold := goal_threshold
		end
end
