note
	description: "Proportional integrated derivative controller"
	author: "Ferran Pallarès"
	date: "01.10.2015"

class
	TARGET_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (stop_sig: separate STOP_SIGNALER; K_p: REAL_64; K_i: REAL_64; K_d: REAL_64; target_thresh: REAL_64; min_time_inc: INTEGER_64)
			-- Create controller given the attributes.
		do
			create trigonometry
			stop_signaler := stop_sig

			Kp := k_p
			Ki := k_i
			Kd := k_d
			target_threshold := target_thresh
			min_time_increment := min_time_inc
		end

feature -- Constants

	Kp: REAL_64
		-- Proportional gain for PID.

	Ki: REAL_64
		-- Integral gain for PID.

	Kd: REAL_64
		-- Derivative gain for PID.

	target_threshold: REAL_64
		-- Target considered reached when robot is at this distance.

	min_time_increment: INTEGER_64
		-- Minimum lapse of time (in msec) among which PID corrections are calculated.

feature -- Access

	prev_time: INTEGER_64
			-- Last time (in msec) the algorithm was run.

	move_to_target (pid_sig: separate TARGET_SIGNALER; x_target: REAL_64; y_target: REAL_64; odom_sig: separate ODOMETRY_SIGNALER; d_drive: separate DIFFERENTIAL_DRIVE)
			-- Move towards the specified point in space.
		require
			not pid_sig.is_target_reached
			and ({TIMER}.current_time_millis - prev_time) >= min_time_increment
		local
			x: REAL_64
			y: REAL_64
			theta: REAL_64
			current_time: INTEGER_64
			target_theta: REAL_64
			error: REAL_64
			delta_time: REAL_64
			output: REAL_64
		do
			current_time := {TIMER}.current_time_millis

			if sep_is_stop_requested (stop_signaler) then
				d_drive.stop

			elseif prev_time = 0 then
				prev_time := current_time

			else
				x := odom_sig.x
				y := odom_sig.y
				theta := odom_sig.theta

				target_theta := trigonometry.atan2 (y_target - y, x_target - x)
				error := target_theta - theta

				delta_time := (current_time - prev_time)/1000

				acc_error := acc_error + error*delta_time
				output := Kp*error + Ki*acc_error + Kd*(error - prev_error)/delta_time

				d_drive.set_velocity (d_drive.max_linear_speed, output)

				prev_time := current_time
			end
		end

	stop_when_reached (pid_sig: separate TARGET_SIGNALER; x_target: REAL_64; y_target: REAL_64; odom_sig: separate ODOMETRY_SIGNALER; d_drive: separate DIFFERENTIAL_DRIVE)
			-- Stop moving when target reached.
		require
			not pid_sig.is_target_reached
			and ({TIMER}.current_time_millis - prev_time) >= min_time_increment
		local
			x: REAL_64
			y: REAL_64
			current_time: INTEGER_64
		do
			current_time := {TIMER}.current_time_millis

			if sep_is_stop_requested (stop_signaler) then
				d_drive.stop

			elseif prev_time = 0 then
				prev_time := current_time

			else
				x := odom_sig.x
				y := odom_sig.y

				if ({DOUBLE_MATH}.dabs(y_target - y) < target_threshold and {DOUBLE_MATH}.dabs(x_target - x) < target_threshold) then
					d_drive.stop
					pid_sig.set_target_reached (True)
					io.put_string ("GOAL REACHED%N")
					--
						io.put_string ("%NCurrent: x = ")
						io.put_double (x)
						io.put_string (", y = ")
						io.put_double (y)
						io.put_string (", theta = ")
						io.put_double (odom_sig.theta)
						io.put_string ("%N")
					--
				end

				prev_time := current_time
			end
		end

feature {NONE} -- Implementation

	trigonometry: TRIGONOMETRY_MATH
			-- Used for atan2 calculation.

	prev_error: REAL_64
			-- Previous error.

	acc_error: REAL_64
			-- Accumulated error.

end
