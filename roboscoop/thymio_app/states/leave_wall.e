note
	description: "Summary description for {LEAVE_WALL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	LEAVE_WALL

inherit
	STATE

create
	make
feature {NONE} --Atributes
	target: POINT_2D

	orientation_controller: PID
	speed_controller: NON_LINEAR_SPEED
	time_handler: TIME_PARSER
	math: TRIGONOMETRY_MATH

	pid_parser: PID_GAIN_PARSER



feature -- Initializer
	make
		-- Make
		do
			create math
			create pid_parser.make_with_path ("pid_gains.txt")
			create time_handler.start (0.0)
			create speed_controller.make_with_speed (0.05)
			create orientation_controller.make_with_gains (0.6, 0.0, 0.0)

			create target.make
		end


feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
		do
			drive.set_velocity (speed_controller.get_output, orientation_controller.get_output)
		end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
		do
			leds.set_to_blue
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
		local
			error: REAL_64
		do

			error := math.atan2(target.get_y - t_sig.get_pose.get_position.get_y, target.get_x - t_sig.get_pose.get_position.get_x) - t_sig.get_pose.get_orientation
			error := math.atan2 (math.sine (error), math.cosine (error))
			time_handler.set_time(t_sig.get_timestamp)
			if time_handler.get_sampling_rate > 0 then
				orientation_controller.set_sampling (time_handler.get_sampling_rate)
				orientation_controller.set_error (error)

				speed_controller.set_angular_velocity (orientation_controller.get_output)
			end
		end


	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
		local
			current_point: POINT_2D
		do
			io.put_string ("target: " + target.get_string + " currant: x: " + t_sig.get_pose.get_position.get_x.out + " y: " + t_sig.get_pose.get_position.get_y.out + "%N")
			if {DOUBLE_MATH}.dabs (r_sig.sensors[1].range) < 0.04 or {DOUBLE_MATH}.dabs (r_sig.sensors[5].range) < 0.04 then
					if r_sig.is_obstacle_mostly_at_left then
						t_sig.set_follow_wall_counter_clockwise
					else
						t_sig.set_follow_wall_clockwise
					end
			elseif t_sig.get_goal.get_euclidean_distance (t_sig.get_pose.get_position) < t_sig.get_d_min and
				target.get_euclidean_distance (t_sig.get_pose.get_position) < 0.1 then
				t_sig.set_go_to_goal
			end
		end

	set_target (t: POINT_2D)
			-- set target in global coordinates
		do
			target := t
		ensure
			target_set: target = t
		end

end
