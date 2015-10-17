note
	description: "Summary description for {POSE_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	POSE_2D

create
	make, make_with_pose, make_with_coordinates

feature {NONE} -- Implementation

	point: POINT_2D
	orientation: REAL_64

feature	-- Initialization

	make
			-- Initialize object
		do
			create point.make
			orientation := 0
		end

	make_with_pose (p: POINT_2D; phi: REAL_64)
			-- Initialize object with a given position and orientation
		do
			set_position(p)
			set_orientation(phi)
		end

	make_with_coordinates(x, y, phi: REAL_64)
			-- Initialize pose from coordinates
		do
			make_with_pose(create {POINT_2D}.make_with_coordinates (x, y), phi)
		end


feature{ANY} -- Accesors

	get_position: POINT_2D
			-- Return current position
		do
			Result := point
		end

	set_position (p: POINT_2D)
			-- Set new position
		do
			point := p
		ensure
			point_set: point = p
		end

	get_orientation: REAL_64
			-- Get current oreintation
		do
			Result := orientation
		end

	set_orientation (phi: REAL_64)
			-- Set new orientation
		do
			orientation := phi
		ensure
			orientation_set: orientation = phi
		end

end
