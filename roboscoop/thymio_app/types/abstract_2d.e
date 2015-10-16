note
	description: "Summary description for {ABSTRACT_2D}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	ABSTRACT_2D


create
	make, make_with_coordinates, make_from_vector_3d_msg

feature{NONE} -- Attributes
	x, y: REAL_64

feature{ANY} -- Initializaiton
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

	make_from_vector_3d_msg(msg: VECTOR_3D_MSG)
		-- Init point fom VECTOR_3D_MSG
	do
		make_with_coordinates ( msg.x, msg.y )
	end

feature {ANY} -- Accesors
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

	get_vector_3d_msg: VECTOR_3D_MSG
	do
		Result := create {VECTOR_3D_MSG}.make_with_values (x, y, 0.0)
	end


end
