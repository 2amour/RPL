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
					r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Move robot towards goal when no obstacle is sensed.
		require
			not t_sig.is_go_to_goal_pending
			not s_sig.is_stop_requested
		do
			t_sig.clear_all_pendings
			t_sig.set_is_go_to_goal_pending (True)
		end

	follow_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Move robot arround sensed obstacle.
		require
			is_obstacle_sensed (r_g)
			not t_sig.is_leave_obstacle_pending
			not s_sig.is_stop_requested
		do
			if not t_sig.is_follow_obstacle_pending then
				t_sig.set_is_follow_obstacle_pending (True)
				-- Save entry point and reset minimum distance.
			end
			-- Record distance to goal and update minimum.
		end

	leave_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Move robot towards a sensend safe point in space.
		require
			is_closer_safe_point_sensed
			not t_sig.is_leave_obstacle_pending
			not s_sig.is_stop_requested
		do
			t_sig.set_is_leave_obstacle_pending (True)

			if t_sig.goal.get_euclidean_distance (get_current_pose (o_sig).get_position) < t_sig.minimum_distance_to_goal then
				t_sig.set_is_go_to_goal_pending (False)
			end
		end

	reached_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
					r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Proceed when the robot has reached the goal.
		require
			is_goal_reached (t_sig, o_sig)
			not t_sig.is_reached_goal_pending
			not s_sig.is_stop_requested
		do

		end

	unreachable_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Proceed when the goal is unreachable.
		require
			is_goal_unreachable (t_sig, o_sig)
			not t_sig.is_unreachable_goal_pending
			not s_sig.is_stop_requested
		do
			d_d.stop
		end

feature {TANGENT_BUG_BEHAVIOUR} -- Implementation

	is_obstacle_sensed (r_g: separate RANGE_GROUP): BOOLEAN
			-- Whether an obstacle is sensed.
		do
			Result := r_g.is_obstacle
		end

	is_goal_reached (t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Whether the goal has been reached.
		do
			Result := t_sig.goal.get_euclidean_distance (get_current_pose (o_sig).get_position) < t_sig.point_reached_threshold
		end

	is_closer_safe_point_sensed: BOOLEAN
			-- Whether there is a safe sensed point closer to the goal than minimum recorded distance.
		do
		end

	is_goal_unreachable (t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Determines whether the robot has entered the point where it started following an obstacle.
		local
			robot_position: POINT_2D
			distance_robot_to_obstacle_entry_point: REAL_64
		do
			robot_position := create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y)
			distance_robot_to_obstacle_entry_point := robot_position.get_euclidean_distance (t_sig.obstacle_entry_point)

			if distance_robot_to_obstacle_entry_point >= t_sig.revisited_point_threshold then
				t_sig.set_has_left_obstacle_entry_point (True)	-- TODO follow wall and leave wall have to handle this to set it to false each time we deal with another obstacle!
			end

			Result := distance_robot_to_obstacle_entry_point < t_sig.point_reached_threshold and t_sig.has_left_obstacle_entry_point
		end

feature {NONE} -- Implementation

	get_current_pose (o_sig: separate ODOMETRY_SIGNALER): POSE_2D
			-- Return current pose.
		local
			pose: POSE_2D
		do
			create pose.make_with_coordinates (o_sig.x, o_sig.y, o_sig.theta)
			Result := pose
		end
end
