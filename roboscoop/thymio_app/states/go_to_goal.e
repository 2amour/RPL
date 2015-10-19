note
	description: "Summary description for {GO_TO_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	GO_TO_GOAL

inherit
	TANGENT_BUG_STATE
create
	make

feature{NONE}
	orientation_controller: PID
	speed_controller: NON_LINEAR_SPEED
	time_handler: TIME_PARSER
	math: TRIGONOMETRY_MATH

	pid_parser: PID_GAIN_PARSER

feature
	make
			-- Make.
		do
			create math
			create pid_parser.make
			create time_handler.start (0.0)
			create speed_controller.make
			create orientation_controller.make_with_gains (pid_parser.Kp, pid_parser.Ki, pid_parser.Kd)
		end

feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
			-- <Precursor>
		do
			leds.set_to_yellow
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
			-- <Precursor>
		local
			error: REAL_64
		do

			error := math.atan2(t_sig.get_goal.get_y - t_sig.get_pose.get_position.get_y, t_sig.get_goal.get_x - t_sig.get_pose.get_position.get_x) - t_sig.get_pose.get_orientation
			error := math.atan2 (math.sine (error), math.cosine (error))
			time_handler.set_time(t_sig.get_timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
			-- <Precursor>
		do
			if r_sig.is_obstacle_in_front then
				if r_sig.is_obstacle_mostly_at_left then
					t_sig.set_follow_wall_counter_clockwise
				else
					t_sig.set_follow_wall_clockwise
				end
			end
			if t_sig.get_goal.get_euclidean_distance (t_sig.get_pose.get_position) < 0.05 then -- TODO HARDCODING
				t_sig.set_at_goal
			end
		end

end
