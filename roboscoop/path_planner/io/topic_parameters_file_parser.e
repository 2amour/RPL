note
	description: "Topic file parser."
	author: "Sebastian Curi"
	date: "06.11.2015"

deferred class
	TOPIC_PARAMETERS_FILE_PARSER

feature {ANY} -- Access
	parse_file (file_path: STRING): TOPIC_PARAMETERS
			-- Parse file with path `file_path'.
		deferred
		end

end
