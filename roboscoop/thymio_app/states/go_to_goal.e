note
	description: "Summary description for {GO_TO_GOAL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	GO_TO_GOAL

inherit
	STATE
create
	make_with_goal

feature{NONE}
	goal: POINT_2D
	orientation_controller: PID
	speed_controller: NON_LINEAR_SPEED
	time_handler: TIME_PARSER
	math: TRIGONOMETRY_MATH

	pid_parser: PID_GAIN_PARSER

feature
	make_with_goal(g: POINT_2D)
		-- make with goal
	do
		create math
		create pid_parser.make
		create time_handler.start (0.0)
		create speed_controller.make
		goal := g
		create orientation_controller.make_with_gains (pid_parser.Kp, pid_parser.Ki, pid_parser.Kd)
	end

feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
	do
		drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
	do
		leds.set_to_yellow
	end

	set_readings(odometry_signaler:separate ODOMETRY_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
	local
		error: REAL_64
	do
		error := math.atan2(goal.get_y - odometry_signaler.y, goal.get_x - odometry_signaler.x) - odometry_signaler.theta
		time_handler.set_time(odometry_signaler.timestamp)
		if time_handler.get_sampling_rate > 0 then
			orientation_controller.set_sampling (time_handler.get_sampling_rate)
			orientation_controller.set_error (error)

			speed_controller.set_angular_velocity (orientation_controller.get_output)
		end
	end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
	do
		if r_sig.is_obstacle_in_front then
			t_sig.set_follow_wall
		end
	end

end
