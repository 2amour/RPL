note
	description: "A 3D point implementation."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	POINT

create
	make_default, make_with_coordinates, make_from_separate, make_from_msg

feature {NONE} -- Initialization

	make_default
			-- Make `Current' with default values.
		do
			x := 0
			y := 0
			z := 0
		end

	make_with_coordinates (a_x, a_y, a_z: REAL_64)
			-- Make `Current' with given values.
		do
			x := a_x
			y := a_y
			z := a_z
		end

	make_from_separate (other: separate like Current)
			-- Make `Current' with given values.
		do
			x := other.x
			y := other.y
			z := other.z
		end

	make_from_msg (msg: separate POINT_MSG)
			-- Make `Current' with given values.
		do
			x := msg.x
			y := msg.y
			z := msg.z
		end


feature {ANY} -- Access

	x: REAL_64
			-- x coordinate.
	y: REAL_64
			-- y coordinate.
	z: REAL_64
			-- z coordinate.

	get_msg: POINT_MSG
			-- Get point_msg associated to this point.
		do
			Result := create {POINT_MSG}.make_with_values (x, y, z)
		end

end
