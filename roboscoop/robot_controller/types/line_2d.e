note
	description: "Line in 2D."
	author: "Sebastian Curi"
	date: "18.10.15"

class
	LINE_2D

create
	make, make_with_points

feature {NONE} -- Initialization

	make
			-- Make with zero coordinates.
		do
			create p_1.make
			create p_2.make
		end

	make_with_points (p1, p2: POINT_2D)
			-- Line from 2 points.
		do
			p_1 := p1
			p_2 := p2
		end

feature -- Access

	get_vector: VECTOR_2D
			-- Get vector with line direction starting starting at first point.
		do
			Result := create {VECTOR_2D}.make_with_coordinates (p_2.get_x - p_1.get_x, p_2.get_y - p_1.get_y)
		end

	get_distance_from_point (p: POINT_2D): REAL_64
			-- Get distance between line and point.
		local
			r, v: VECTOR_2D
		do
			v :=  get_vector.get_perpendicular
			r :=  create {VECTOR_2D}.make_from_points (p_1, p)
			Result := {DOUBLE_MATH}.dabs (v.get_unitary.dot (r))
		end

feature {NONE} -- Implementation

	p_1, p_2: POINT_2D
		-- Point in 2D.
end
