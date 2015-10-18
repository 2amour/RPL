note
	description: "PID gains parser."
	author: "ferran_antoni_sebastian"
	date: "18.10.15"

class
	PID_GAIN_PARSER

inherit
	FILE_PARSER

create
	make_with_path, make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'
		local
			paths_parser: PATHS_PARSER
		do
			create paths_parser.make
			read_file (paths_parser.pid_gain_file_path)
		end

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		do
			read_file(path)
		end

feature -- Access

	Kp, Ki, Kd: REAL_64
			-- PID gains.

feature {NONE} -- Implementation

	read_file (f_path: STRING)
			-- Read file
		do
			file_path := f_path
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
end
