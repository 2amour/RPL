note
	description: "Summary description for {LINE_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	LINE_2D

create
	make, make_with_points

feature {NONE} -- Implementation
	p_1: POINT_2D
	p_2: POINT_2D

feature -- Initializations
	make
			-- make with zero coordinates
		do
			create p_1.make
			create p_2.make
		end

	make_with_points (p1, p2: POINT_2D)
			-- Init line from 2 points.
		do
			p_1 := p1
			p_2 := p2
		end

feature --Accesors

	get_vector: VECTOR_2D
			-- Get vector with line direction starting starting at first point.
		do
			Result := create {VECTOR_2D}.make_with_coordinates (p_2.get_x - p_1.get_x, p_2.get_y - p_1.get_y) -- TODO - Larger X point - smaller X point; If equal smaller Y - larger Y
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


end
