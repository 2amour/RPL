note
	description: "General class for types in 2D"
	author: "Sebastian Curi"
	date: "18.10.2015"

class
	GEOMETRY_2D

create
	make, make_with_coordinates, make_from_vector_3d_msg, make_from_separate

feature {ANY} -- Initialization

	make
			-- Initialize point to 0, 0 coordinates.
		do
			x := 0
			y := 0
		end

	make_from_separate (other: separate GEOMETRY_2D)
			-- Initialize object from separate object.
		do
			x := other.get_x
			y := other.get_y
		end

	make_with_coordinates (new_x, new_y: REAL_64)
			-- Initialize point from given coordinates.
		do
			x := new_x
			y := new_y
		ensure
			set_x: x = new_x
			set_y: y = new_y
		end

	make_from_vector_3d_msg (msg: VECTOR_3D_MSG)
			-- Initialize point from VECTOR_2D_MSG.
		do
			make_with_coordinates (msg.x, msg.y)
		end

feature -- Access

	set_coordinates (new_x, new_y: REAL_64)
			-- Set new new_x, new_y coordinates.
		do
			make_with_coordinates (new_x, new_y)
		end

	get_x: REAL_64
			-- Return x coordinate.
		do
			Result := x
		end

	get_y: REAL_64
			-- Return y coordinate.
		do
			Result := y
		end

	get_vector_3d_msg: VECTOR_3D_MSG
			-- Get a vector_3d_msg from coordinates x and y.
		do
			Result := create {VECTOR_3D_MSG}.make_with_values (x, y, 0.0)
		end

	get_scaled alias "*" (scalar: REAL_64): like Current
			-- Get original abstract_2d scaled by `scalar'.
		do
			Result := create {like Current}.make_with_coordinates (x*scalar, y*scalar)
		end

	add alias "+" (other: like Current): like Current
			-- The result abstract_2d is the addition between the original abstract_2d and `other'.
		do
			Result := create {like Current}.make_with_coordinates (x + other.get_x, y + other.get_y)
		end

	sub alias "-" (other: like Current): like Current
			-- The result abstract_2d is the addition between the original abstract_2d and `other'.
		do
			Result := create {like Current}.make_with_coordinates (x - other.get_x, y - other.get_y)
		end

	get_string: STRING_8
			-- Abstract2D to STRING_8.
		do
			Result := "x: " + x.out + " y: " + y.out
		end

feature {NONE} -- Implementation

	x, y: REAL_64
		-- Coordinates.
end
