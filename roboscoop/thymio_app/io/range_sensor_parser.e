note
	description: "Summary description for {RANGE_SENSOR_PARSER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	RANGE_SENSOR_PARSER
inherit
	FILE_PARSER
	--ARGUMENTS

create
	make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		local
			x: REAL_64
			y: REAL_64
			phi: REAL_64
			i: INTEGER
			transform: TRANSFORM_2D
		do
			create transform.make
			create transforms.make_filled (transform, 1, 7)
			create file.make_open_read (file_path)
			i := 1
			from file.start
			until file.off
			loop
				file.read_double
				x := file.last_double.item
				file.read_double
				y := file.last_double.item
				file.read_double
				phi := file.last_double.item

				create transform.make_with_offsets(x, y, phi)
				transforms.put (transform, i)

				i := i + 1
			end

			file.close
		end

feature {NONE} --Initialization
	file_path: STRING = "./sensor_coordinates.txt"
		-- File path in system.
feature -- Access
	transforms: ARRAY[TRANSFORM_2D]

end
