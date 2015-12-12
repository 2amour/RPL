note
	description: "Class for receiving an Empty msg."
	author: "Sebastian Curi"
	date: "12.12.2015"


deferred class
	EMPTY_LISTENER

feature -- Access

	update_state (msg: separate EMPTY_MSG)
			-- Update current state with the values from `msg'.
		deferred
		end

end
