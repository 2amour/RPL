note
	description: "Summary description for {POINT_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	POINT_2D

create
	make, make_with_coordinates

feature{NONE} -- Attributes
	x, y: REAL_64

feature -- Initializaiton
	make
		-- Init point to 0, 0 coordinates
	do
		x := 0
		y := 0
	end

	make_with_coordinates ( new_x, new_y: REAL_64 )
		-- Init point to (new_x, new_y)
	do
		x := new_x
		y := new_y
	ensure
		set_x: x = new_x
		set_y: y = new_y
	end

feature -- Accesors
	set_coordinates( new_x, new_y: REAL_64)
		-- Set new new_x, new_y coordinates
	do
		make_with_coordinates ( new_x, new_y)
	end

	get_x: REAL_64
		-- Return x coordinate
	do
		Result := x
	end

	get_y: REAL_64
		-- Return y coordinate
	do
		Result := y
	end

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
