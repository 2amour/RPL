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
			-- Make a vector starting at `p1' pointing towards `p2'.
		do
			x := p2.get_x - p1.get_x
			y := p2.get_y - p1.get_y
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
			Result := create {VECTOR_2D}.make_with_coordinates (-y, x) -- TODO - NOTE, if vector points RIGHT, perpendicular points UP
		end

	dot (other: VECTOR_2D): REAL_64
			-- dot product between vectors.
		do
			Result := other.get_x * x + other.get_y * y
		end
end
