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

	g_th: REAL_64
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
				file.read_word
				file.read_double
				inspect file.last_word
				when "goal_th" then
					g_th := file.last_double
				when "" then
					Ki := file.last_double
				when "" then
					Kd := file.last_double
				when "" then
								Kd := file.last_double
				else
					io.putstring ("Character not recognized")
				end
			end
			file.close
		end
end
