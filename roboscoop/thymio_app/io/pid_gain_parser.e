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
	make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		do
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
		end

feature {NONE} --Initialization
	file_path: STRING = "pid_gains.txt"
		-- File path in system.
feature -- Access
	Kp, Ki, Kd: REAL_64
end
