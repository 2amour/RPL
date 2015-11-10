note
	description: "Closed loop orientation controller."
	author: "Ferran Pallarès"
	date: "07.11.2015"

class
	ORIENTATION_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (stop_sig: separate STOP_SIGNALER; odometry_sig: separate ODOMETRY_SIGNALER; diff_drive: separate DIFFERENTIAL_DRIVE; pid_parameters: separate PID_PARAMETERS)
			-- Create self with attributes.
		do
			create math
			create time_handler.start (0.0)
			create speed_controller.make
			create orientation_controller.make_with_gains (pid_parameters.kp, pid_parameters.ki, pid_parameters.kd)

			stop_signaler := stop_sig
			odometry_signaler := odometry_sig
			differential_drive := diff_drive
		end

feature -- Access

	update_drive (o_sig: separate ORIENTATION_SIGNALER)
			-- Update differential drive using orientation value.
		local
			error: REAL_64
		do
			error := o_sig.orientation - get_normalized_angle (get_current_orientation (odometry_signaler))

			time_handler.set_time(get_timestamp (odometry_signaler))

			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end

			if not o_sig.is_stopped then
				set_velocity (differential_drive, speed_controller.get_output, orientation_controller.get_output)
			else
				stop (differential_drive)
			end
		end

feature {NONE} -- Implementation

	odometry_signaler: separate ODOMETRY_SIGNALER
			-- Robot odometry information.

	differential_drive: separate DIFFERENTIAL_DRIVE
			-- Robot differential drive.

	orientation_controller: PID
			-- Closed loop orientation controller.

	speed_controller: NON_LINEAR_SPEED
			-- Speed controller in terms of angular speed.

	time_handler: TIME_PARSER
			-- Time handler.

	math: TRIGONOMETRY_MATH
			-- Object for math computation.

	get_timestamp (odometry_sig: separate ODOMETRY_SIGNALER): REAL_64
			-- Get `odometry_sig' message timestamp.
		do
			Result := odometry_sig.timestamp
		end

	get_current_orientation (odometry_sig: separate ODOMETRY_SIGNALER): REAL_64
			-- Get current robot orientation
		do
			Result := odometry_sig.theta
		end

	set_velocity (diff_drive: separate DIFFERENTIAL_DRIVE; linear_velocity, angular_velocity: REAL_64)
			-- Set `diff_drive' velocity.
		do
			diff_drive.set_velocity (linear_velocity, angular_velocity)
		end

	stop (diff_drive: separate DIFFERENTIAL_DRIVE)
			-- Stop `diff_drive'.
		do
			diff_drive.stop
		end

	get_normalized_angle (theta: REAL_64): REAL_64
			-- Change the interval of definition of `theta' from 0 to pi, -pi to 0.
		local
			theta_out: REAL_64
		do
			if (theta < 0) then
				theta_out := theta + (2*math.pi)
			else
				theta_out := theta
			end
			if theta_out > math.pi then
				theta_out := theta_out - 2*math.pi
			end
			Result := theta_out
		end
end
