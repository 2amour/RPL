note
	description: "Summary description for {PID_GAIN_PARSER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	PID_GAIN_PARSER
inherit
	FILE_PARSER
	--ARGUMENTS

create
	make_with_path

feature {NONE} -- Initialization

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		do
			file_path := path
			create file.make_open_read (file_path)

			from file.start;
			until file.off
			loop
				file.read_character
				file.read_double
				inspect file.last_character
				when 'p' then
					Kp := file.last_double
				when 'i' then
					Ki := file.last_double
				when 'd' then
					Kd := file.last_double
				else
					io.putstring ("Character not recognized")
				end
			end
			file.close

			debug
				io.put_string ("p: " + Kp.out + ", i: " + Ki.out + ", d: " + Kd.out + "%N")
			end
		end

feature -- Access
	Kp, Ki, Kd: REAL_64
end
