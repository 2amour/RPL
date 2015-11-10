note
	description: "Summary description for {ORIENTATION_SIGNALER}." -- TODO - set
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	ORIENTATION_SIGNALER

create
	make

feature {NONE} -- Initialization

	make
			-- Create self.
		do
			is_stopped := True
		end

feature -- Access

	orientation: REAL_64
			-- Target orientation.

	is_stopped: BOOLEAN
			-- Whether robot should be stopped.

	follow_orientation (theta: REAL_64)
			-- Start heading towards `orientation'.
		do
			orientation := theta
			is_stopped := False
		end

	stop
			-- Stop orientation controller.
		do
			is_stopped := True
		end

end
