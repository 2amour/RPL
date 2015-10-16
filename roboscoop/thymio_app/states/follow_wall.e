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
		new_state: TANGENT_BUG_STATE
		v_leave_point: POINT_2D
	do
		--create v_leave_point.make_with_coordinates (o_sig.x + v_leave * {DOUBLE_MATH}.cosine(o_sig.theta),
		--											o_sig.y + v_leave * {DOUBLE_MATH}.sine  (o_sig.theta))


		--if r_sig.has_obstacle (0.0) and
		--	t_sig.get_goal.get_euclidean_distance ( v_leave_point ) < t_sig.get_d_min then

		--	create new_state.make_with_state ({TANGENT_BUG_STATE}.leave_wall)
		--	t_sig.set_state (new_state)

		--end

	end

end
