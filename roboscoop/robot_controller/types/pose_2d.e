note
	description: "Pose in 2D."
	author: "Sebastian Curi"
	date: "18.10.2015"

class
	POSE_2D

create
	make, make_with_pose, make_with_coordinates, make_from_separate

feature	-- Initialization

	make
			-- Initialize object.
		do
			create point.make
			create math
			orientation := 0
		end

	make_with_pose (p: separate POINT_2D; phi: REAL_64)
			-- Initialize object with a given position and orientation.
		do
			create math
			set_position (p)
			set_orientation (math.atan2 (math.sine(phi), math.cosine (phi)))
		end

	make_from_separate (other: separate POSE_2D)
		do
			create math
			make_with_coordinates (other.get_position.get_x, other.get_position.get_y, other.get_orientation)
		end

	make_with_coordinates (x, y, phi: REAL_64)
			-- Initialize pose from coordinates.
		do
			create math
			make_with_pose (create {POINT_2D}.make_with_coordinates (x, y), math.atan2 (math.sine(phi), math.cosine (phi)))
		end

feature -- Access

	math: TRIGONOMETRY_MATH
			-- Trigonometry library

	get_position: POINT_2D
			-- Return current position.
		do
			Result := point
		end

	set_position (p: separate POINT_2D)
			-- Set new position.
		do
			create point.make_from_separate (p)
		ensure
			point_set: point = p
		end

	get_orientation: REAL_64
			-- Get current oreintation.
		do
			Result := orientation
		end

	set_orientation (phi: REAL_64)
			-- Set new orientation.
		do
			orientation := math.atan2 (math.sine(phi), math.cosine (phi))
		end

feature {NONE} -- Implementation

	point: POINT_2D
			-- Point in 2D.

	orientation: REAL_64
			-- Orientation.
end
