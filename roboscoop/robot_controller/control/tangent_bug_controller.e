note
	description: "Controls robot during tangent bug behaviour."
	author: "Ferran Pallarès"
	date: "06.11.15"

class
	TANGENT_BUG_CONTROLLER

inherit
	CANCELLABLE_CONTROL_LOOP

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (stop_sig: separate STOP_SIGNALER)
			-- Create controller given the attributes.
		do
			stop_signaler := stop_sig
		end

feature {TANGENT_BUG_BEHAVIOUR} -- Access

	go_to_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
					r_g: separate RANGE_GROUP; lift: separate LIFTABLE; orientation_sig: separate ORIENTATION_SIGNALER)
			-- Move robot towards goal when no obstacle is sensed.
		require
			(not t_sig.is_minimum_distance_recorded) or t_sig.is_closer_than_minimum_distance (o_sig)
			not t_sig.is_go_to_goal_blocked
			not s_sig.is_stop_requested
		local
			orientation: REAL_64
			math: TRIGONOMETRY_MATH
		do
			create math
			orientation := math.atan2 (t_sig.goal.get_y - o_sig.y, t_sig.goal.get_x - o_sig.x)
			orientation := math.atan2 (math.sine (orientation), math.cosine (orientation))
			orientation_sig.follow_orientation (orientation)

			io.put_string ("GO TO GOAL%N")
			t_sig.clear_all_blockeds
			t_sig.set_is_go_to_goal_blocked (True)
		end

	follow_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; orientation_sig: separate ORIENTATION_SIGNALER)
			-- Move robot arround sensed obstacle.
		require
			t_sig.is_obstacle_sensed (r_g)
			not t_sig.is_leave_obstacle_blocked
			not s_sig.is_stop_requested
		local
			current_position: POINT_2D
			current_distance: REAL_64
		do
			io.put_string ("OBSTACLE%N")
			current_position := get_current_position (o_sig)
			current_distance := t_sig.goal.get_euclidean_distance (current_position)
			if not t_sig.is_follow_obstacle_blocked then
				t_sig.set_is_follow_obstacle_blocked (True)
				-- Save entry point and reset minimum distance.
				t_sig.set_obstacle_entry_point (current_position)
				t_sig.set_has_left_obstacle_entry_point (False)
				t_sig.set_minimum_distance_to_goal (current_distance)
			end
			-- Record distance to goal and update minimum.
			if current_distance < t_sig.minimum_distance_to_goal then
				t_sig.set_minimum_distance_to_goal (current_distance)
			end
		end

	leave_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; orientation_sig: separate ORIENTATION_SIGNALER)
			-- Move robot towards a sensend safe point in space.
		require
			t_sig.is_closer_safe_point_sensed (o_sig, r_g)
			t_sig.is_follow_obstacle_blocked
			not t_sig.is_leave_obstacle_blocked and t_sig.is_leave_obstacle_blocked
			not s_sig.is_stop_requested
		local
			orientation: REAL_64
			math: TRIGONOMETRY_MATH
			safe_point: POINT_2D
		do
			create safe_point.make_from_separate (r_g.get_closest_safe_point_in_front (t_sig.goal))
			create math
			orientation := math.atan2 (safe_point.get_y - o_sig.y, safe_point.get_x - o_sig.x)
			orientation := math.atan2 (math.sine (orientation), math.cosine (orientation))
			orientation_sig.follow_orientation (orientation)

			io.put_string ("LEAVE OBSTACLE%N")
			io.put_string((create {POINT_2D}.make_from_separate (r_g.get_closest_safe_point_in_front (t_sig.goal))).get_string + "%N")

			t_sig.set_is_leave_obstacle_blocked (True)
			t_sig.set_is_go_to_goal_blocked (False)

--			if t_sig.goal.get_euclidean_distance (get_current_position (o_sig)) < t_sig.minimum_distance_to_goal then
--				t_sig.set_is_go_to_goal_blocked (False)
--			end
		end

	reached_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
					r_g: separate RANGE_GROUP; lift: separate LIFTABLE; orientation_sig: separate ORIENTATION_SIGNALER)
			-- Proceed when the robot has reached the goal.
		require
			t_sig.is_goal_reached (o_sig)
			not t_sig.is_reached_goal_blocked
			not s_sig.is_stop_requested
		do
			orientation_sig.stop
			io.put_string ("REACHED GOAL%N")
			t_sig.set_is_reached_goal_blocked (True)
		end

	unreachable_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; orientation_sig: separate ORIENTATION_SIGNALER)
			-- Proceed when the goal is unreachable.
		require
			t_sig.is_goal_unreachable (o_sig)
			not t_sig.is_unreachable_goal_blocked
			not s_sig.is_stop_requested
		do
			orientation_sig.stop
			io.put_string ("UNREACHABLE GOAL%N")
		end

feature {NONE} -- Implementation

	get_current_position (o_sig: separate ODOMETRY_SIGNALER): POINT_2D
			-- Return current pose.
		local
			point: POINT_2D
		do
			create point.make_with_coordinates (o_sig.x, o_sig.y)
			Result := point
		end
end
