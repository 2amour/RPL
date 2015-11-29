note
	description: "Parser for path_planner topics."
	author: "Sebastian Curi"
	date: "06.11.2015"


class
	PATH_PLANNER_TOPICS_PARSER

inherit
	TOPIC_PARAMETERS_FILE_PARSER

feature {ANY} -- Access.

	parse_file (file_path: separate STRING): PATH_PLANNER_TOPICS_PARAMETERS
			-- Parse file with path `file_path'.
		local
			params: PATH_PLANNER_TOPICS_PARAMETERS
			file: PLAIN_TEXT_FILE
			key: STRING
		do
			create params.make_default

			create file.make_open_read (create {STRING}.make_from_separate (file_path))
			from
				file.start
			until
				file.off
			loop
				file.read_word_thread_aware
				key := file.last_string

				if key.is_equal ("map:") then
					file.read_word_thread_aware
					params.set_map (file.last_string)
				elseif key.is_equal("node_name:") then
					file.read_word_thread_aware
					params.set_node_name (file.last_string)
				elseif key.is_equal("path:") then
					file.read_word_thread_aware
					params.set_path (file.last_string)
				elseif key.is_equal("start:") then
					file.read_word_thread_aware
					params.set_start (file.last_string)
				elseif key.is_equal("goal:") then
					file.read_word_thread_aware
					params.set_goal (file.last_string)
				elseif not key.is_empty then
					io.putstring ("Parser error while parsing file '" + file.name + "': Key '" + key + "' not recognized%N")
				end
			end
			file.close

			Result := params
		end

end

