note
	description: "Summary description for {MISSION_PLANNER_TOPICS_PARSER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	MISSION_PLANNER_TOPICS_PARSER

inherit
	TOPIC_PARAMETERS_FILE_PARSER

feature {ANY} -- Access.

	parse_file (file_path: STRING): MISSION_PLANNER_TOPICS_PARAMETERS
			-- Parse file with path `file_path'.
		local
			path, map, target, odometry, start, goal: STRING_8
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			path := ""
			map := ""
			target := ""
			odometry := ""
			start := ""
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
				elseif key.is_equal("map:") then
					file.read_word_thread_aware
					create map.make_from_string (file.last_string)
				elseif key.is_equal("target:") then
					file.read_word_thread_aware
					create target.make_from_string (file.last_string)
				elseif key.is_equal("odometry:") then
					file.read_word_thread_aware
					create odometry.make_from_string (file.last_string)
				elseif key.is_equal("start:") then
					file.read_word_thread_aware
					create start.make_from_string (file.last_string)
				elseif key.is_equal("goal:") then
					file.read_word_thread_aware
					create goal.make_from_string (file.last_string)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file_path + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := create {MISSION_PLANNER_TOPICS_PARAMETERS}.make_with_attributes (path, map, target, odometry, start, goal)
		end

end
