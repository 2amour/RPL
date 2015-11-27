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
			create vector_to_goal.make
			create angular_sections_reached.make_filled (False, 0, 49)
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

				tangent_bug_sig.current_pose.get_position.set_coordinates (o_sig.x, o_sig.y) -- TODO expanded class POSE?
				tangent_bug_sig.current_pose.set_orientation (o_sig.theta) -- TODO - Hola? get pose?
				tangent_bug_sig.set_timestamp (o_sig.timestamp)

				vector_to_goal.make_from_points (tangent_bug_sig.goal, tangent_bug_sig.current_pose.get_position)
				angle_to_goal := change_interval (vector_to_goal.get_angle, vector_to_goal.get_x)
				if has_turned_around_goal (angle_to_goal)  then
					tangent_bug_sig.set_unreachable_goal
				end

				tangent_bug_sig.state.set_readings(tangent_bug_sig, r_sig)
				tangent_bug_sig.state.update_velocity (drive)
				tangent_bug_sig.state.update_state(tangent_bug_sig, o_sig, r_sig)
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
			t_sig.reset_goal_coordinates (goal_sig.data.x, goal_sig.data.y)
			goal_sig.set_new_val (False)
		end

feature {NONE} -- Implementation

	angle_to_goal: REAL_64
			-- Angle of vector from robot to goal with respect to the global frame.

	vector_to_goal: VECTOR_2D
			-- Vector to goal from robot with respect to odometry frame.

	angular_sections_reached: ARRAY[BOOLEAN]
			-- Angular sections with respect to goal that are crossed by robot

	change_interval (theta: REAL_64; x_vector: REAL_64): REAL_64
			-- Change the interval of definition of `theta', now it will be from 0 to 2*pi.
		local
			theta_res: REAL_64
			math: TRIGONOMETRY_MATH
		do
			theta_res := theta
			create math
			if (x_vector <= 0) then
					theta_res := theta + math.pi
			else
				if (theta <= 0) then
					theta_res := theta + (2 * math.pi)
				end
			end
			Result := theta_res
		end

	has_turned_around_goal (theta: REAL_64): BOOLEAN
			-- Check if robot has turned 360 degrees with respect to the goal.
		local
			math: TRIGONOMETRY_MATH
			angular_section: REAL_64
			truncated: INTEGER_32
			vector_of_trues: ARRAY[BOOLEAN]
		do
			create math
			create vector_of_trues.make_filled (True, 0, 49)
			angular_section := 50 * (theta / (2 * math.pi))
			if angular_section.truncated_to_integer >= 50 then
				truncated := 49
			elseif angular_section.truncated_to_integer <= 0  then
				truncated := 0
			else
				truncated := angular_section.truncated_to_integer
			end
			angular_sections_reached.put (True, truncated)
			if (angular_sections_reached.is_equal (vector_of_trues)) then
				Result := True
			else
				Result := False
			end
		end
end
