note
	description: "PID gains parser."
	author: "ferran_antoni_sebastian"
	date: "18.10.15"

class
	TRESHOLD_PARSER

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
			read_file (paths_parser.treshold_file_path)
		end

	make_with_path (path: STRING)
			-- Initialization for `Current'.
		do
			read_file(path)
		end

feature -- Access

	g_th, y_rob, a_vel, t_th, d_wall: REAL_64
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
				when 'g' then
					g_th := file.last_double
				when 'a' then
					y_rob := file.last_double
				when 'b' then
					a_vel := file.last_double
				when 'c' then
					t_th := file.last_double
				when 'd' then
					d_wall := file.last_double
				else
					io.putstring ("Character not recognized")
				end
			end
			file.close
		end
end
