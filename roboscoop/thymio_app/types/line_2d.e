note
	description: "Summary description for {LINE_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	LINE_2D

create
	make, make_with_points

feature{NONE} -- Attributes
	p_1: POINT_2D
	p_2: POINT_2D

feature -- Initializations
	make
		-- make with zero coordinates
	do
		create p_1.make
		create p_2.make
	end

	make_with_points(p1, p2: POINT_2D)
	do
		p_1 := p1
		p_2 := p2
	end

feature --Accesors
	get_vector: VECTOR_2D
		-- Get Vector formed by the line
	do
		Result := create {VECTOR_2D}.make_with_coordinates (p_1.get_x - p_2.get_x, p_1.get_y - p_2.get_y)
	end
end
