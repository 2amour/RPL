note
	description: "Summary description for {FOLLOW_WALL}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	FOLLOW_WALL

inherit
	STATE
create
	make_with_v_leave
feature{NONE} -- Atributes
	v_leave : REAL_64

feature -- Initializer
	make_with_v_leave(v: REAL_64)
	require
		positive_v: v > 0
	do
		v_leave := v
	ensure
		set_v: v_leave = v
	end

feature
	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
	do
		drive.stop
	end

	update_leds(leds: separate RGB_COLOR_ACTUATOR)
	do
		leds.set_to_red
	end

	set_readings(odometry_signaler:separate ODOMETRY_SIGNALER; range_signaler:separate THYMIO_RANGE_GROUP)
	do
	end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP)
	local
		v_leave_point: separate POINT_2D
		vector_to_goal: VECTOR_2D

	do
		create vector_to_goal.make_from_points(create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y), t_sig.get_goal)


		create v_leave_point.make_with_coordinates (o_sig.x + v_leave * {DOUBLE_MATH}.cosine(vector_to_goal.get_angle),
													o_sig.y + v_leave * {DOUBLE_MATH}.sine  (vector_to_goal.get_angle))


		if r_sig.has_obstacle (vector_to_goal.get_angle) and
			t_sig.get_goal.get_euclidean_distance ( v_leave_point ) < t_sig.get_d_min then

			t_sig.set_leave_wall

		end

	end

end
