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
		do

		end
end
