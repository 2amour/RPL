note
	description: "Parser class for robot_controller ros topics."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	ROBOT_CONTROLLER_TOPICS_PARSER

inherit
	TOPIC_PARAMETERS_FILE_PARSER

feature {ANY} -- Access.
	parse_file (file_path: STRING): ROBOT_CONTROLLER_TOPIC_PARAMETERS
			-- Parse file with path `file_path'.
		local
			name, path, pose, mission_odometry, sensed_obstacles, goal: STRING_8
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			name := ""
			path := ""
			pose := ""
			mission_odometry := ""
			sensed_obstacles := ""
			goal := ""

			create file.make_open_read (file_path)
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
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := create {ROBOT_CONTROLLER_TOPIC_PARAMETERS}.make_with_attributes (name, path, pose, mission_odometry, sensed_obstacles, goal)
		end

end

