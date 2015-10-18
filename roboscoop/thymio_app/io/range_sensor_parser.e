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
	make, make_with_path

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		local
			paths_parser: PATHS_PARSER
		do
			create paths_parser.make
			read_file (paths_parser.range_sensor_file_path)
		end

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		do
			read_file (path)
		end

feature -- Access
	transforms: ARRAY[TRANSFORM_2D]

	read_file (f_path: STRING)
			-- Reads file
		local
			x: REAL_64
			y: REAL_64
			phi: REAL_64
			i: INTEGER
			transform: TRANSFORM_2D
		do
			create transform.make
			create transforms.make_filled (transform, 1, 7)
			file_path := f_path
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
			debug
				across transforms as t
				loop
					io.put_string ("x= " + t.item.x.out + ", y= " + t.item.y.out + ", phi = " + t.item.phi.out + "%N")
				end
			end
		end
end
