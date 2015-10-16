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
		do
			create transforms.make_empty
			create file.make_open_read (file_path)

			from file.start
			until file.off
			loop
				file.read_double
				x := file.last_double.item
				file.read_double
				y := file.last_double.item
				file.read_double
				phi := file.last_double.item


				transforms.put (create {TRANSFORM_2D}.make_with_offsets(x, y, phi), i)

				i := i + 1
			end

			file.close
		end

feature {NONE} --Initialization
	file_path: STRING = "io/target_constants.txt"
		-- File path in system.
feature -- Access
	transforms: ARRAY[TRANSFORM_2D]

end
