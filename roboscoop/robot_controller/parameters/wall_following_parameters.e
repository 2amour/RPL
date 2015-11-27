note
	description: "Wall following parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	WALL_FOLLOWING_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty wall following parameters object.
		do
			create safe_outer_corner_turn_offset.make
		end

	make_with_attributes (wall_following_desired_wall_distance: REAL_64; wall_following_outer_corner_angular_velocity: REAL_64; wall_following_linear_velocity: REAL_64;
							wall_following_safe_outer_corner_turn_offset: POINT_2D; wall_following_safe_outer_corner_turn_offset_threshold: REAL_64;
							wall_following_safe_leaving_wall_vertical_distance: REAL_64)
			-- Create wall following parameters object with attributes.
		do
			desired_wall_distance := wall_following_desired_wall_distance
			outer_corner_angular_velocity := wall_following_outer_corner_angular_velocity
			linear_velocity := wall_following_linear_velocity
			safe_outer_corner_turn_offset := wall_following_safe_outer_corner_turn_offset
			safe_outer_corner_turn_offset_threshold := wall_following_safe_outer_corner_turn_offset_threshold
			safe_leaving_wall_vertical_distance := wall_following_safe_leaving_wall_vertical_distance
		end

	make_from_separate (other: separate WALL_FOLLOWING_PARAMETERS)
			-- Create wall following parameters object from separate other.
		do
			desired_wall_distance := other.desired_wall_distance
			outer_corner_angular_velocity := other.outer_corner_angular_velocity
			linear_velocity := other.linear_velocity
			create safe_outer_corner_turn_offset.make_from_separate (other.safe_outer_corner_turn_offset)
			safe_outer_corner_turn_offset_threshold := other.safe_outer_corner_turn_offset_threshold
			safe_leaving_wall_vertical_distance := other.safe_leaving_wall_vertical_distance
		end

feature -- Access

	desired_wall_distance: REAL_64
			-- Desired distance between the robot and the followed wall.

	outer_corner_angular_velocity: REAL_64
			-- Angular velocity for the robot to turn an outer corner.

	linear_velocity: REAL_64
			-- Linear velocity for the robot to follow an obstacle.

	safe_outer_corner_turn_offset: POINT_2D
			-- Position offset for a safe outer corner turn.

	safe_outer_corner_turn_offset_threshold: REAL_64
			-- Threshold for considering when the position offset for a safe outer corner is reached.

	safe_leaving_wall_vertical_distance: REAL_64
			-- Minimum local vertical distance between the robot and the followed wall to leave it safely.

	set_desired_wall_distance (wall_following_desired_wall_distance: REAL_64)
			-- Setter for `desired_wall_distance'.
		do
			desired_wall_distance := wall_following_desired_wall_distance
		end

	set_outer_corner_angular_velocity (wall_following_outer_corner_angular_velocity: REAL_64)
			-- Setter for `outer_corner_angular_velocity'.
		do
			outer_corner_angular_velocity := wall_following_outer_corner_angular_velocity
		end

	set_linear_velocity (wall_following_linear_velocity: REAL_64)
			-- Setter for `wall_following_linear_velocity'.
		do
			linear_velocity := wall_following_linear_velocity
		end

	set_safe_outer_corner_turn_offset (wall_following_safe_outer_corner_turn_offset: POINT_2D)
			-- Setter for `safe_outer_corner_turn_offset'
		do
			safe_outer_corner_turn_offset := wall_following_safe_outer_corner_turn_offset
		end

	set_safe_outer_corner_turn_offset_threshold (wall_following_safe_outer_corner_turn_offset_threshold: REAL_64)
			-- Setter for `safe_outer_corner_turn_offset_threshold'.
		do
			safe_outer_corner_turn_offset_threshold := wall_following_safe_outer_corner_turn_offset_threshold
		end

	set_safe_leaving_wall_vertical_distance (wall_following_safe_leaving_wall_vertical_distance: REAL_64)
			-- Setter for `safe_leaving_wall_vertical_distance'
		do
			safe_leaving_wall_vertical_distance := wall_following_safe_leaving_wall_vertical_distance
		end

end
