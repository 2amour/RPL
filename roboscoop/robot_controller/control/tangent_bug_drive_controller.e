note
	description: "Controls robot movement during tangent bug behaviour."
	author: "ferran_antoni_sebastian"
	date: "18.10.15"

class
	TANGENT_BUG_DRIVE_CONTROLLER

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

	update_velocity(tangent_bug_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate THYMIO_RANGE_GROUP;
					g_sig: separate LIFTABLE; s_sig: separate STOP_SIGNALER;
					drive: separate DIFFERENTIAL_DRIVE)
					-- Update velocity settings.
		do
			if s_sig.is_stop_requested then
				drive.stop
			else

				tangent_bug_sig.get_pose.get_position.set_coordinates (o_sig.x, o_sig.y) -- TODO expanded class POSE?
				tangent_bug_sig.get_pose.set_orientation (o_sig.theta)
				tangent_bug_sig.set_timestamp (o_sig.timestamp)

				vector_to_goal.make_from_points (tangent_bug_sig.get_goal, tangent_bug_sig.get_pose.get_position)
				angle_to_goal := change_interval (vector_to_goal.get_angle, vector_to_goal.get_x)
			 	if has_turned_around_goal (angle_to_goal)  then
			 		tangent_bug_sig.set_unreachable_goal
			 	end

				tangent_bug_sig.get_state.set_readings(tangent_bug_sig, r_sig)
				tangent_bug_sig.get_state.update_velocity (drive)
				tangent_bug_sig.get_state.update_state(tangent_bug_sig, o_sig, r_sig)
			end
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
