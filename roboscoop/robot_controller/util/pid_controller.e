note
	description: "Generic PID controller."
	author: "Sebastian Curi"
	date: "15.10.2015"

class
	PID_CONTROLLER

create
	make_with_gains

feature -- Initialization

	make_with_gains (p, i, d: REAL_64)
			-- Initialize with Controller gains.
		do
			Kp := p
			Ki := i
			Kd := d
		ensure
			set_proportional: Kp = p
			set_integral: Ki = i
			set_derivative: Kd = d
		end

feature -- Access

	set_threshold (t: REAL_64)
			-- Set treshold for integrator wind-up.
		require
			valid_threshold: t > 0
		do
			integrating_threshold := t
		end


	set_sampling (dt: REAL_64)
			-- Set sampling interval.
		require
			valid_sampling: dt > 0
		do
			delta_t := dt
		end


	set_error (e: REAL_64)
			-- Set error.
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
			-- Return controller output.
		do
			Result := output
		end

	reset
			-- Reset controller status.
		do
			perror := 0.0
			ierror := 0.0
			derror := 0.0
			delta_t := 0.0
		end

feature {NONE} -- Implementation

	Kp, Ki, Kd: REAL_64
			-- PID gains.

	integrating_threshold: REAL_64
			-- Threshold for integrator wind-up.

	perror, ierror, derror: REAL_64
			-- Proportional, integral and derivative error.

	delta_t: REAL_64
			-- Time increment over which the error is calculated.

	output: REAL_64
			-- PID output.
end
