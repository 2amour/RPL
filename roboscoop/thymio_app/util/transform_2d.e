note
	description: "TRANSFORM_2D initializes a frame with a 2D offset."
	author: "ferran_antoni_sebastian"
	date: "18.10.2015"

class
	TRANSFORM_2D

create
	make, make_with_offsets

feature -- Initialize

	make
			-- Make empty.
		do
			x := 0
			y := 0
			phi := 0
			cos_phi := {DOUBLE_MATH}.cosine (phi)
			sin_phi := {DOUBLE_MATH}.sine (phi)
		end

	make_with_offsets (new_x, new_y, new_phi: REAL_64)
			-- Make with new values.
		do
			x := new_x
			y := new_y
			phi := new_phi
			cos_phi := {DOUBLE_MATH}.cosine (phi)
			sin_phi := {DOUBLE_MATH}.sine (phi)
		ensure
			set_x: x = new_x
			set_y: y = new_y
			set_phi: phi = new_phi
		end

feature -- Access

	x, y, phi: REAL_64
			-- Position and angle of coordinate frame.

	cos_phi, sin_phi: REAL_64
			-- Cosinus and sinus of angle of coordinate frame.

	get_heading: REAL_64
			-- Get heading of coordinate frame.
		do
			Result := phi
		end

	project_to_parent (local_point: POINT_2D ): POINT_2D
			-- Project a point in local coordinates to global coordinates.
		local
			parent_point: POINT_2D
		do
			create parent_point.make_with_coordinates (x + local_point.get_x * cos_phi - local_point.get_y * sin_phi ,
													   y + local_point.get_x * sin_phi + local_point.get_y * cos_phi)
			Result := parent_point
		end

	project_from_parent (parent_point: POINT_2D): POINT_2D
			-- Project a point in global coordinates to local coordinates.
		local
			local_point: POINT_2D
		do
			create local_point.make_with_coordinates ( (parent_point.get_x - x) * cos_phi + (parent_point.get_y - y)* sin_phi ,
													  -(parent_point.get_x - x) * sin_phi + (parent_point.get_y - y)* cos_phi)
			Result := local_point
		end
end
