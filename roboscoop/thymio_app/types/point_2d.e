note
	description: "Summary description for {POINT_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	POINT_2D

inherit
	ABSTRACT_2D

create
	make, make_with_coordinates, make_from_vector_3d_msg

feature --Accesors
	get_euclidean_distance(p: POINT_2D): REAL_64
		-- Get Euclidean distance between point p and this point
	do
		Result:= {DOUBLE_MATH}.sqrt ( (p.get_x - x)*(p.get_x - x) + (p.get_y - y)*(p.get_y - y) )
	end

	get_manhattan_distance(p: POINT_2D): REAL_64
		-- Get Manhattan distance between point p and this point
	do
		Result:= {DOUBLE_MATH}.dabs ( p.get_x - x) + {DOUBLE_MATH}.dabs (p.get_y - y)
	end

end
