note
	description: "Tangent bug parameters bag that groups different types of parameters."
	author: "Ferran Pallarès"
	date: "22.10.2015"

class
	TANGENT_BUG_PARAMETERS_BAG

inherit
	PARAMETERS_BAG

create
	make_with_attributes

feature {NONE} -- Implementation

	make_with_attributes (tangent_bug_goal_parameters: separate GOAL_PARAMETERS;
							tangent_bug_wall_following_parameteres: separate WALL_FOLLOWING_PARAMETERS;
							tangent_bug_go_to_goal_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS;
							tangent_bug_follow_wall_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS;
							tangent_bug_leave_wall_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS)
			-- Make tangent bug parameters bag from attributes.
		do
			goal_parameters := tangent_bug_goal_parameters
			wall_following_parameters := tangent_bug_wall_following_parameteres
			go_to_goal_pose_controller_parameters := tangent_bug_go_to_goal_pose_controller_parameters
			follow_wall_pose_controller_parameters := tangent_bug_follow_wall_pose_controller_parameters
			leave_wall_pose_controller_parameters := tangent_bug_leave_wall_pose_controller_parameters

		end

feature -- Access

	goal_parameters: separate GOAL_PARAMETERS
			-- Goal parameters.

	wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS
			-- Wall following parameters.

	go_to_goal_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS
			-- Go to goal state pose controller parameters.

	follow_wall_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS
			-- Follow wall state pose controller parameters.

	leave_wall_pose_controller_parameters: separate POSE_CONTROLLER_PARAMETERS
			-- Leave wall state pose controller parameters.
end
