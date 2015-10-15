note
	description: "TRANSFORM_2D initializes a frame with a 2D offset."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TRANSFORM_2D

create
	make, make_with_offsets

feature -- Attributes
	x, y, phi: REAL_64
	cos_phi, sin_phi: REAL_64

feature --Initialize
	make
		-- Make empty
	do
		x := 0
		y := 0
		phi := 0

		cos_phi := {DOUBLE_MATH}.cosine (phi)
		sin_phi := {DOUBLE_MATH}.sine (phi)
	end

	make_with_offsets(new_x, new_y, new_phi: REAL_64)
		-- Make with new values
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

feature -- Accesors
	get_heading: REAL_64
		-- get heading of coordinate frame
	do
		Result := phi
	end

	project_to_global( local_point: POINT_2D ): POINT_2D
		-- Project a point in local coordinates to global coordinates
	local
		global_point: POINT_2D
	do
		create global_point.make_with_coordinates (x + local_point.get_x * cos_phi - local_point.get_y * sin_phi ,
												   y + local_point.get_x * sin_phi + local_point.get_y * cos_phi)

		Result := global_point
	end

	project_from_global( global_point: POINT_2D ): POINT_2D
		-- Project a point in global coordinates to local coordinates
	local
		local_point: POINT_2D
	do
		create local_point.make_with_coordinates ( (global_point.get_x - x) * cos_phi + (global_point.get_y - y)* sin_phi ,
												  -(global_point.get_x - x) * sin_phi + (global_point.get_y - y)* cos_phi)

		Result := local_point
	end


end
