note
	description: "Class with thymio specific topics file parser."
	author: "Sebastian Curi"
	date: "15-12-2015"

class
	THYMIO_TOPIC_PARAMETERS_FILE_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature -- Initialization.

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make_default
		end

feature -- Access

	parse_file (file_path: separate STRING)
			-- Parse file.
		local

			file: PLAIN_TEXT_FILE
			key: STRING
			file_checker: FILE_CHECKER
			f_path: STRING
		do
			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				 file.open_read
				 from file.start
				 until file.off
				 loop
				 	file.read_word_thread_aware
					key := file.last_string
				 	if key.is_equal ("odometry:") then
						file.read_word_thread_aware
						last_parameters.set_odometry (file.last_string.twin)
					elseif key.is_equal ("range_sensors:") then
						file.read_word_thread_aware
						last_parameters.set_range_sensors (file.last_string.twin)
					elseif key.is_equal ("ground_sensors:") then
						file.read_word_thread_aware
						last_parameters.set_ground_sensors (file.last_string.twin)
					elseif key.is_equal ("velocity:") then
						file.read_word_thread_aware
						last_parameters.set_velocity (file.last_string.twin)
					end
				 end
				 file.close
			else
				is_error_found := True
			end
		end

	last_parameters: THYMIO_TOPIC_PARAMETERS
			-- Parameters parsed.	
end
