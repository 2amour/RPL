note
	description: "Checks whether a file can be open and read."
	author: "Antoni Rosinol Vidal"
	date: "25.11.2015"

class
	FILE_CHECKER

feature -- Access

	check_file (file: PLAIN_TEXT_FILE): BOOLEAN
			-- Checks if file exits and is readable.
		do
			if not file.exists then
			   io.putstring ("error: file '" + file.path.out + "' does not exist%N")
			   Result:= False
			else
			   if not file.is_readable then
			     io.putstring ("error: file '" + file.path.out + "' is not readable.%N")
			     Result:= False
			   else
			     file.open_read
			     Result:= True
			   end
			end
		end
end
