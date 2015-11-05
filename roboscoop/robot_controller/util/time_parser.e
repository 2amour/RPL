note
	description: "Class for timing."
	author: "ferran_antoni_sebastian"
	date: "18.10.2015"

class
	TIME_PARSER

create
	start

feature -- Initialization

	start (t: REAL_64)
			-- Start with a initial 't' time.
		require
			positive_time: t>0
		do
			time := t
			delta_time := 0
		end

feature -- Access

	set_time (t: REAL_64)
			-- Set new time.
		do
			delta_time := t - time
			time := t
		ensure
			time_set: time = t
		end

	get_sampling_rate: REAL_64
			-- Return estimated sampling rate.
		do
			Result := delta_time
		end

feature {NONE} -- Implementation

	time, delta_time: REAL_64
			--  Time and time difference.
end
