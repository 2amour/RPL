note
	description: "Parser class for robot_controller ros topics."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	ROBOT_CONTROLLER_TOPICS_PARSER

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

feature {ANY} -- Access.

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
				 	if key.is_equal ("path:") then
						file.read_word_thread_aware
						last_parameters.set_path (file.last_string.twin)
					elseif key.is_equal ("name:") then
						file.read_word_thread_aware
						last_parameters.set_name (file.last_string.twin)
					elseif key.is_equal ("pose:") then
						file.read_word_thread_aware
						last_parameters.set_pose (file.last_string.twin)
					elseif key.is_equal ("mission_odometry:") then
						file.read_word_thread_aware
						last_parameters.set_mission_odometry (file.last_string.twin)

					elseif key.is_equal ("sensed_obstacles:") then
						file.read_word_thread_aware
						last_parameters.set_sensed_obstacles (file.last_string.twin)
					elseif key.is_equal ("goal:") then
						file.read_word_thread_aware
						last_parameters.set_goal (file.last_string.twin)

					elseif key.is_equal ("circ_led:") then
						file.read_word_thread_aware
						last_parameters.set_circular_leds (file.last_string.twin)
					elseif key.is_equal ("viz_mark:") then
						file.read_word_thread_aware
						last_parameters.set_visualization_marker (file.last_string.twin)

					end
				 end
				 file.close
			else
				is_error_found := True
			end
		end

	last_parameters: ROBOT_CONTROLLER_TOPIC_PARAMETERS
			-- Parsed parameters.

end

