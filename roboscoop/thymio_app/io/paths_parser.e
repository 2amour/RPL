note
	description: "Parser of file with paths of other files"
	author: "ferran_antoni_sebastian"
	date: "17.10.15"

class
	PATHS_PARSER

inherit
	FILE_PARSER
	ARGUMENTS_32

create
	make, make_with_path

feature {NONE} -- Initialization

	make
			-- Initialization for `Current' with file path specified fro command line option -p
		do
			file_path := get_path_from_arguments
			create file.make_open_read (file_path)
			read_file (file)
			file.close
		end

	make_with_path (f_path: STRING)
			-- Initialization for `Current'.
		do
			file_path := f_path
			create file.make_open_read (file_path)
			read_file (file)
			file.close
		end

feature -- Access

	goal_file_path: STRING
			-- Path for the file with the goal

	pid_gain_file_path: STRING
			-- Path for the file with pid gains

	range_sensor_file_path: STRING
			-- Path for the file with range sensors' constants

feature {NONE} -- Implementation

	read_file (f: PLAIN_TEXT_FILE)
			-- Read file
		local
			paths_array: ARRAY [STRING]
		do
			-- Default values for the paths
			goal_file_path := "goals.txt"
			pid_gain_file_path := "pid_gains.txt"
			range_sensor_file_path := "sensor_coordinates.txt"
			create paths_array.make_filled ("", 1, 3)
			from
				f.start
			until
				f.off
			loop
				f.read_integer
				f.read_word
				inspect
					f.last_integer
				when 1 then
					 paths_array.put (f.last_string.twin, 1)
				when 2 then
					 paths_array.put (f.last_string.twin, 2)
				when 3 then
					 paths_array.put (f.last_string.twin, 3)
				else
					if (not f.last_integer.is_equal (0)) then
						io.putstring ("Integer not recognized in the paths file%N")
					end
				end
			end
			goal_file_path := paths_array.at (1)
			pid_gain_file_path := paths_array.at (2)
			range_sensor_file_path := paths_array.at (3)
		end

	get_path_from_arguments: STRING
			-- Get the path specified in command argument
		local
			path: STRING
		do
			path := ""
			if argument_count > 0 then
				path := argument (1).string
			end
			if path.is_empty then
				io.put_string ("Use a file with paths of files with goal, pid_gains and sensor range")
			end
			Result := path
		end
end


