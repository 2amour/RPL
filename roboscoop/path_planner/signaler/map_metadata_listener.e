note
	description: "Class for receiving MAP_METADATA_MSG."
	author: "Sebastian Curi"
	date: "29.10.2015"

deferred class
	MAP_METADATA_LISTENER

feature -- Access

	update_map_metadata (msg: separate MAP_METADATA_MSG)
			-- Update map metadata with the values from `msg'.
		deferred
		end

end
