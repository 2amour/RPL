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
		do

		end

	follow_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Move robot arround sensed obstacle.
		require
		do



		end

	leave_obstacle (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Move robot towards a sensend safe point in space.
		require
		do

		end

	reached_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
					r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Proceed when the robot has reached the goal.
		require
		do

		end

	unreachable_goal (s_sig: separate STOP_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER;
						r_g: separate RANGE_GROUP; lift: separate LIFTABLE; d_d: separate DIFFERENTIAL_DRIVE)
			-- Proceed when the goal is unreachable.
		require
			is_goal_unreachable (t_sig.obstacle_entry_point, o_sig)
		do
			d_d.stop
		end


feature {NONE} -- Implementation

	is_goal_unreachable (obstacle_entry_point: separate POINT_2D; o_sig: separate ODOMETRY_SIGNALER): BOOLEAN
			-- Determines whether the robot has entered the point where it started following an obstacle.
		local
			robot_position: POINT_2D
			distance_robot_to_obstacle_entry_point: REAL_64
		do
			robot_position := create {POINT_2D}.make_with_coordinates (o_sig.x, o_sig.y)
			distance_robot_to_obstacle_entry_point := robot_position.get_euclidean_distance (obstacle_entry_point)

			if distance_robot_to_obstacle_entry_point >= t_sig.revisited_point_threshold then
				has_left_obstacle_entry_point := True				--- TODO follow wall and leave wall have to handle this to set it to false each time we deal with another obstacle!
			end

			Result := distance_robot_to_obstacle_entry_point < t_sig.point_visited_treshold and has_left_obstacle_entry_point
		end


end
