note
	description: "Summary description for {VECTOR_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	VECTOR_2D

inherit
	ABSTRACT_2D
create
	make, make_with_coordinates, make_from_vector_3d_msg, make_from_points

feature

	make_from_points(p1, p2: separate POINT_2D)
			-- Make a vector from points
		do
			x := p1.get_x - p2.get_x
			y := p1.get_y - p2.get_y
		end

feature -- Access

	get_unitary: VECTOR_2D
		-- get unitary vector in the direction of original vector
	local
		magnitude: REAL_64
	do
		magnitude := get_magnitude
		Result := create {VECTOR_2D}.make_with_coordinates (x/magnitude, y/magnitude)
	end

	get_magnitude: REAL_64
		-- get magnitude of vector
	do
		Result := {DOUBLE_MATH}.sqrt (x * x + y * y)
	end

	get_angle: REAL_64
		-- get vector of this angle
	do
		Result := {DOUBLE_MATH}.arc_tangent (y/x)
	end

	get_perpendicular: VECTOR_2D
		-- get perpendicular to this vector
	do
		Result := create {VECTOR_2D}.make_with_coordinates (y, -x)
	end

	get_scaled (scalar: REAL_64): VECTOR_2D
		-- get original vector scaled by `scalar'
	do
		Result := create {VECTOR_2D}.make_with_coordinates (x*scalar, y*scalar)
	end

	add (other: VECTOR_2D): VECTOR_2D
		-- The result vector is the addition between the original vector and `other'
	do
		Result := create {VECTOR_2D}.make_with_coordinates (x + other.get_x, y + other.get_y)
	end

	dot(other: VECTOR_2D): REAL_64
		-- dot product between vectors
	do
		Result := other.get_x * x + other.get_y * y
	end


end
