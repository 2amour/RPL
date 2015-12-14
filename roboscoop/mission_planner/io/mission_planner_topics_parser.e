note
	description: "Parser of topics for mission planner"
	author: "Sebastian Curi"
	date: "30.11.15"

class
	MISSION_PLANNER_TOPICS_PARSER

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
			-- Parse file with path `file_path'.
		local
			file: PLAIN_TEXT_FILE
			key: STRING
			f_path: STRING
			file_checker: FILE_CHECKER
		do
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
					key := file.last_string.twin

					if key.is_equal ("path:") then
						file.read_word_thread_aware
						last_parameters.set_path (file.last_string.twin)
					elseif key.is_equal("node_name:") then
						file.read_word_thread_aware
						last_parameters.set_node_name (file.last_string.twin)
					elseif key.is_equal("map:") then
						file.read_word_thread_aware
						last_parameters.set_map (file.last_string.twin)
					elseif key.is_equal("target:") then
						file.read_word_thread_aware
						last_parameters.set_target (file.last_string.twin)
					elseif key.is_equal("odometry:") then
						file.read_word_thread_aware
						last_parameters.set_odometry (file.last_string.twin)
					elseif key.is_equal("sensed_obstacles:") then
						file.read_word_thread_aware
						last_parameters.set_sensed_obstacle (file.last_string.twin)
					elseif key.is_equal("start:") then
						file.read_word_thread_aware
						last_parameters.set_path_planner_start (file.last_string.twin)
					elseif key.is_equal("goal:") then
						file.read_word_thread_aware
						last_parameters.set_path_planner_goal (file.last_string.twin)
					elseif key.is_equal("planner_map:") then
						file.read_word_thread_aware
						last_parameters.set_planner_map (file.last_string.twin)
					elseif key.is_equal("planner_map_frame:") then
						file.read_word_thread_aware
						last_parameters.set_planner_map_frame (file.last_string.twin)
					elseif key.is_equal("recognize_object:") then
						file.read_word_thread_aware
						last_parameters.set_object_recognition_request (file.last_string.twin)
                    elseif key.is_equal("is_image_recieved:") then
                        file.read_word_thread_aware
                        last_parameters.set_object_recognition_signaler (file.last_string.twin)
                    elseif key.is_equal("req_localization:") then
						file.read_word_thread_aware
						last_parameters.set_localization_request (file.last_string.twin)
                    elseif key.is_equal("is_localized:") then
                        file.read_word_thread_aware
                        last_parameters.set_localization_signaler (file.last_string.twin)
					elseif not key.is_empty then
						io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
						is_error_found := True
					end
				end
				file.close
			else
				is_error_found := True
			end
		end

		last_parameters: MISSION_PLANNER_TOPICS_PARAMETERS
				-- Parameters being parsed.
end
