note
	description: "Class for receiving a Point."
	author: "Sebastian Curi"
	date: "07.11.2015"

deferred class
	POINT_LISTENER

feature -- Access

	update_point (msg: separate POINT_MSG)
			-- Update current state with the values from `msg'.
		deferred
		end
end
