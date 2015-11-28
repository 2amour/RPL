note
	description: "Class for receiving a Marker MSG."
	author: "Sebastian Curi"
	date: "28.11.2015"

deferred class
	MARKER_LISTENER

feature -- Access

	update_marker (msg: separate MARKER_MSG)
			-- Update current state with the values from `msg'.
		deferred
		end

end

