note
	description: "Pose in 2D."
	author: "ferran_antoni_sebastian"
	date: "18.10.2015"

class
	POSE_2D

create
	make, make_with_pose, make_with_coordinates

feature	-- Initialization

	make
			-- Initialize object.
		do
			create point.make
			orientation := 0
		end

	make_with_pose (p: POINT_2D; phi: REAL_64)
			-- Initialize object with a given position and orientation.
		do
			set_position (p)
			set_orientation (phi)
		end

	make_with_coordinates (x, y, phi: REAL_64)
			-- Initialize pose from coordinates.
		do
			make_with_pose (create {POINT_2D}.make_with_coordinates (x, y), phi)
		end

feature -- Access

	get_position: POINT_2D
			-- Return current position.
		do
			Result := point
		end

	set_position (p: POINT_2D)
			-- Set new position.
		do
			point := p
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
			orientation := phi
		ensure
			orientation_set: orientation = phi
		end

feature {NONE} -- Implementation

	point: POINT_2D
			-- Point in 2D.

	orientation: REAL_64
			-- Orientation.
end
