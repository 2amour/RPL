note
    description: "In this state the robot is heading directly towards the goal."
	author: "Ferran Pallarès"
	date: "28.11.2015"

class
	GO_TO_GOAL

inherit
	TANGENT_BUG_STATE

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS)
			-- Create self with attributes.
		do
			create pose_controller.make_with_attributes (pose_controller_parameters.pid_parameters, pose_controller_parameters.nlsc_parameters,
															pose_controller_parameters.turning_angular_speed, pose_controller_parameters.reached_point_threshold,
															pose_controller_parameters.reached_orientation_threshold)
		end

feature

	update_velocity(drive: separate DIFFERENTIAL_DRIVE)
			-- <Precursor>
		do
			pose_controller.update_drive_velocity (drive)
		end

	set_readings(t_sig: separate TANGENT_BUG_SIGNALER; range_signaler:separate RANGE_GROUP)
			-- <Precursor>
		do
			pose_controller.set_target_pose (t_sig.goal)
			pose_controller.set_current_pose (t_sig.current_pose, t_sig.timestamp)
		end

	update_state(t_sig: separate TANGENT_BUG_SIGNALER; o_sig: separate ODOMETRY_SIGNALER; r_sig: separate RANGE_GROUP)
			-- <Precursor>
		do
			if pose_controller.is_target_pose_reached then
				t_sig.set_at_goal
			elseif not pose_controller.is_target_position_reached and r_sig.is_obstacle_in_front then
				if r_sig.is_obstacle_mostly_at_left then
					t_sig.set_follow_wall_counter_clockwise
				else
					t_sig.set_follow_wall_clockwise
				end
			end
		end

feature {NONE} -- Implementation

	pose_controller: POSE_CONTROLLER
			-- Pose controller.
end
