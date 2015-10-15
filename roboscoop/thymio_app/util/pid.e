note
	description: "Summary description for {PID}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	PID
create
	make_with_gains

feature {NONE} -- Attributes
	Kp, Ki, Kd: REAL_64
	integrating_threshold: REAL_64
	perror, ierror, derror: REAL_64
	delta_t: REAL_64
	output: REAL_64

feature -- Initialization
	make_with_gains(p, i, d: REAL_64)
		-- Initialize with Controller gains

	do
		Kp := p
		Ki := i
		Kd := d
	ensure
		set_proportional: Kp = p
		set_integral: Ki = i
		set_derivative: Kd = d
	end

feature --Accesors
	set_threshold(t: REAL_64)
		-- Set integrator treshold for integrator wind-up
	require
		valid_threshold: t > 0
	do
		integrating_threshold := t
	end


	set_sampling(dt: REAL_64)
		-- Set sampling interval
	require
		valid_sampling: dt > 0
	do
		delta_t := dt
	end


	set_error(e: REAL_64)
		-- Set error
	do
		if {DOUBLE_MATH}.dabs(e) < integrating_threshold then
			ierror := ierror + e*delta_t
		else
			ierror := 0.0
		end

		derror := (e - perror) / delta_t
		perror := e

		output := Kp * perror + Ki * ierror + Kd * derror
	end


	get_output: REAL_64
		-- Return Controller Output
	do
		Result := output
	end



end
