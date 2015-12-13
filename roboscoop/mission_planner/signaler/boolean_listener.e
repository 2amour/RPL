note
	description: "Class for receiving a Bool msg."
	author: "Sebastian Curi"
	date: "12.12.2015"

deferred class
	BOOLEAN_LISTENER

feature -- Access

	update_state (msg: separate BOOLEAN_MSG)
			-- Update current state with the values from `msg'.
		deferred
		end

end

