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

	update_velocity(tangent_bug_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP;
					g_sig: separate LIFTABLE; s_sig: separate STOP_SIGNALER;
					drive: separate DIFFERENTIAL_DRIVE)
			-- Update velocity settings.
		do
			if s_sig.is_stop_requested then
				drive.stop
			else
				tangent_bug_sig.set_pose (create {separate POSE_2D}.make_with_coordinates (o_sig.x, o_sig.y, o_sig.theta))
				tangent_bug_sig.set_timestamp (o_sig.timestamp)

				tangent_bug_sig.state.set_readings (tangent_bug_sig, r_sig)
				tangent_bug_sig.state.update_velocity (drive)
				tangent_bug_sig.state.update_state (tangent_bug_sig, o_sig, r_sig)
			end
		end

	publish_odometry (o_sig: separate ODOMETRY_SIGNALER; o_pub: separate ODOMETRY_PUBLISHER)
			-- Publish odometry
		require
			o_sig.timestamp > o_pub.timestamp
		do
			o_pub.publish_odometry (o_sig)
		end

	publish_obstacles (r_sig: separate RANGE_GROUP; o_pub: separate POINT_PUBLISHER; o_sig: separate ODOMETRY_SIGNALER)
			-- Publish odometry
		require
			r_sig.is_obstacle_in_front
		local
			idx: INTEGER_32
			point: POINT_2D
			transform: TRANSFORM_2D
		do
			from idx := r_sig.sensors.lower
			until idx > r_sig.sensors.upper
			loop
				if r_sig.sensors[idx].is_valid_range then
					create transform.make_with_offsets (r_sig.transforms[idx].x, r_sig.transforms[idx].y, r_sig.transforms[idx].get_heading)
					create point.make_from_separate (transform.project_to_parent (create {POINT_2D}.make_with_coordinates (r_sig.sensors.at (idx).range, 0)))
					create transform.make_with_offsets (o_sig.x, o_sig.y, o_sig.theta)
					o_pub.publish_point2D (transform.project_to_parent (point))
				end
				idx := idx + 1
			end
		end

	update_goal (goal_sig: separate POINT_SIGNALER; t_sig: separate TANGENT_BUG_SIGNALER)
			-- Update goal coordinates.
		require
			goal_sig.is_new_val
		do
			t_sig.set_goal (goal_sig.data.x, goal_sig.data.y)
			goal_sig.set_new_val (False)
		end
end
