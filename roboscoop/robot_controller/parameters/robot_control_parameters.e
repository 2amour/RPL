note
	description: "Parser for robot behaviour parameters."
	author: "Sebastian Curi"
	date: "13/10/12"

class
	ROBOT_CONTROL_PARAMETERS

inherit
	PARAMETERS

create
	make, make_from_separate

feature {NONE} -- Initiaization
	make
			-- Create default robot control parameters object.
		do
			create initial_goal.make
			goal_threshold := 0.0
			create wall_following_parameters.make
			create go_to_goal_pose_controller_parameters.make
			create follow_wall_pose_controller_parameters.make
			create leave_wall_pose_controller_parameters.make

		end

	make_from_separate (other: separate ROBOT_CONTROL_PARAMETERS)
			-- Create robot control parameters from separate other.
		do
			create initial_goal.make_from_separate (other.initial_goal)
			goal_threshold := other.goal_threshold
			create wall_following_parameters.make_from_separate (other.wall_following_parameters)

			create go_to_goal_pose_controller_parameters.make_from_separate (other.go_to_goal_pose_controller_parameters)
			create follow_wall_pose_controller_parameters.make_from_separate (other.follow_wall_pose_controller_parameters)
			create leave_wall_pose_controller_parameters.make_from_separate (other.leave_wall_pose_controller_parameters)

		end

feature -- Access

	initial_goal: POSE_2D
			-- Initial goal setter.

	set_initial_pose (a_pose: separate POSE_2D)
			-- Set initial goal.
		do
			initial_goal := create {POSE_2D}.make_from_separate (a_pose)
		end

	goal_threshold: DOUBLE
			-- Goal threshold.

	set_goal_threshold (a_val: DOUBLE)
			-- Set goal threshold.
		do
			goal_threshold := a_val
		end

	wall_following_parameters: WALL_FOLLOWING_PARAMETERS
			-- Parameters for wall following behaviour.

	set_wall_following_parameters (other: separate WALL_FOLLOWING_PARAMETERS)
			-- Set wall following parameters.
		do
			wall_following_parameters := create {WALL_FOLLOWING_PARAMETERS}.make_from_separate (other)
		end

	go_to_goal_pose_controller_parameters: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for go to goal

	set_go_to_goal_pose_controller_parameters (other: separate POSE_CONTROLLER_PARAMETERS)
			-- Set go-to-goal controller parameters.
		do
			go_to_goal_pose_controller_parameters := create {POSE_CONTROLLER_PARAMETERS}.make_from_separate (other)
		end

	follow_wall_pose_controller_parameters: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for follow wall

	set_follow_wall_pose_controller_parameters (other: separate POSE_CONTROLLER_PARAMETERS)
			-- Set follow-wall controller parameters.
		do
			follow_wall_pose_controller_parameters := create {POSE_CONTROLLER_PARAMETERS}.make_from_separate (other)
		end

	leave_wall_pose_controller_parameters: POSE_CONTROLLER_PARAMETERS
			-- Pose controller parameters for leave wall

	set_leave_wall_pose_controller_parameters (other: separate POSE_CONTROLLER_PARAMETERS)
			-- Set leave-wall controller parameters.
		do
			leave_wall_pose_controller_parameters := create {POSE_CONTROLLER_PARAMETERS}.make_from_separate (other)
		end

end
