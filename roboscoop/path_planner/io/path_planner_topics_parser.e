note
	description: "Parser for path_planner topics."
	author: "Sebastian Curi"
	date: "06.11.2015"


class
	PATH_PLANNER_TOPICS_PARSER

inherit
	PARAMETERS_FILE_PARSER

create
	make

feature {NONE} -- Initialization.

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
					key := file.last_string

					if key.is_equal ("map:") then
						file.read_word_thread_aware
						last_parameters.set_map (file.last_string)
					elseif key.is_equal("node_name:") then
						file.read_word_thread_aware
						last_parameters.set_node_name (file.last_string)
					elseif key.is_equal("path:") then
						file.read_word_thread_aware
						last_parameters.set_path (file.last_string)
					elseif key.is_equal("start:") then
						file.read_word_thread_aware
						last_parameters.set_start (file.last_string)
					elseif key.is_equal("goal:") then
						file.read_word_thread_aware
						last_parameters.set_goal (file.last_string)
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

		last_parameters: PATH_PLANNER_TOPICS_PARAMETERS
end

