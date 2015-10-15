note
	description: "Class for parsing TARGET_BEHAVIOUR related constants."
	author: "Ferran Pallarès"
	date: "06.10.2015"

class
	TARGET_CONSTANTS

inherit
	FILE_PARSER

create
	make

feature {NONE} -- Initialization

	make
			-- Initialization for `Current'.
		do
			create file.make_open_read (file_path)

			file.read_word file.read_double Kp := file.last_double
			file.read_word file.read_double Ki := file.last_double
			file.read_word file.read_double Kd := file.last_double
			file.read_word file.read_double target_threshold := file.last_double
			file.read_word file.read_integer_64 min_time_increment := file.last_integer_64

			file.close
		end

feature {NONE} -- Constants

	file_path: STRING = "io/target_constants.txt"
		-- File path in system.

feature -- Access

	Kp: REAL_64
		-- Proportional gain for PID.

	Ki: REAL_64
		-- Integral gain for PID.

	Kd: REAL_64
		-- Derivative gain for PID.

	target_threshold: REAL_64
		-- Target considered reached when robot is at this distance.

	min_time_increment: INTEGER_64
		-- Minimum lapse of time (in msec) among which PID corrections are calculated.

end
