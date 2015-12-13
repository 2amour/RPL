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

feature {NONE} -- Implementation.

	make
			-- Create current.
		do
			is_error_found := False
			create last_parameters.make_default
		end

feature {ANY} -- Access.

	parse_file (file_path: separate STRING)
			-- Parse file with path `file_path'.
		local
			name, path, pose, mission_odometry, sensed_obstacles, goal: STRING_8
			circl_led, viz_marker: STRING_8
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
			name := ""
			path := ""
			pose := ""
			mission_odometry := ""
			sensed_obstacles := ""
			goal := ""
			circl_led := ""
			viz_marker := ""

			create f_path.make_from_separate (file_path)
			create file.make (f_path)
			create file_checker

			if file_checker.check_file (file) then
				file.open_read
				from
					file.start
				until
					file.off
				loop
					file.read_word_thread_aware
					key := file.last_string

					if key.is_equal ("path:") then
						file.read_word_thread_aware
						create path.make_from_string (file.last_string)
					elseif key.is_equal("name:") then
						file.read_word_thread_aware
						create name.make_from_string (file.last_string)
					elseif key.is_equal("pose:") then
						file.read_word_thread_aware
						create pose.make_from_string (file.last_string)
					elseif key.is_equal("mission_odometry:") then
						file.read_word_thread_aware
						create mission_odometry.make_from_string (file.last_string)
					elseif key.is_equal("sensed_obstacles:") then
						file.read_word_thread_aware
						create sensed_obstacles.make_from_string (file.last_string)
					elseif key.is_equal("goal:") then
						file.read_word_thread_aware
						create goal.make_from_string (file.last_string)
					elseif key.is_equal("circ_led:") then
						file.read_word_thread_aware
						create circl_led.make_from_string (file.last_string)
					elseif key.is_equal("viz_mark:") then
						file.read_word_thread_aware
						create viz_marker.make_from_string (file.last_string)
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + f_path + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
			else
				is_error_found := True
			end
			create last_parameters.make_with_attributes (name, path, pose, mission_odometry, sensed_obstacles, goal, circl_led, viz_marker)
		end

	last_parameters: ROBOT_CONTROLLER_TOPIC_PARAMETERS

end

