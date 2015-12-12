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

	parse_file (file_path: separate STRING): MISSION_PLANNER_TOPICS_PARAMETERS
			-- Parse file with path `file_path'.
		local
			--node_name, path, map, target, odometry, obstacles, start, goal, planner_map: STRING
			file: PLAIN_TEXT_FILE
			key: STRING
			topics: MISSION_PLANNER_TOPICS_PARAMETERS
		do
			create topics.make_default
			create file.make_open_read (create {STRING}.make_from_separate (file_path))
			from
				file.start
			until
				file.off
			loop
				file.read_word_thread_aware
				key := file.last_string

				if key.is_equal ("path:") then
					file.read_word_thread_aware
					topics.set_path (file.last_string)
				elseif key.is_equal("node_name:") then
					file.read_word_thread_aware
					topics.set_node_name (file.last_string)
				elseif key.is_equal("map:") then
					file.read_word_thread_aware
					topics.set_map (file.last_string)
				elseif key.is_equal("target:") then
					file.read_word_thread_aware
					topics.set_target (file.last_string)
				elseif key.is_equal("odometry:") then
					file.read_word_thread_aware
					topics.set_odometry (file.last_string)
				elseif key.is_equal("sensed_obstacles:") then
					file.read_word_thread_aware
					topics.set_sensed_obstacle (file.last_string)
				elseif key.is_equal("start:") then
					file.read_word_thread_aware
					topics.set_path_planner_start (file.last_string)
				elseif key.is_equal("goal:") then
					file.read_word_thread_aware
					topics.set_path_planner_goal (file.last_string)
				elseif key.is_equal("planner_map:") then
					file.read_word_thread_aware
					topics.set_planner_map (file.last_string)
				elseif key.is_equal("planner_map_frame:") then
					file.read_word_thread_aware
					topics.set_planner_map_frame (file.last_string)
				elseif key.is_equal("recognize_object:") then
					file.read_word_thread_aware
					topics.set_object_recognition_request (file.last_string)
				elseif key.is_equal("is_image_recieved:") then
					file.read_word_thread_aware
					topics.set_object_recognition_signaler (file.last_string)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := topics
		end

end
