note
	description: "Summary description for {TIME_PARSER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	TIME_PARSER
create
	start

feature{NONE} -- Attributes
	time, delta_time: REAL_64


feature --Initialization
	start(t: REAL_64)
		-- Start with a initial 't' time
	require
		positive_time: t>0
	do
		time := t
		delta_time := 0
	end


feature -- Accesors
	set_time(t: REAL)
		-- Set new time
	do
		delta_time := t - time
		time := t
	ensure
		time_set: time = t
	end

	get_sampling_rate: REAL_64
		-- Return estimated sampling rate
	do
		Result := delta_time
	end

end
