note
	description: "Files parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	FILES_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty files parameters object.
		do
			create ros_topics_file_path.make_empty
			create goal_parameters_file_path.make_empty
			create go_to_goal_pid_parameters_file_path.make_empty
			create follow_wall_pid_parameters_file_path.make_empty
			create leave_wall_pid_parameters_file_path.make_empty
			create go_to_goal_nlsc_parameters_file_path.make_empty
			create follow_wall_nlsc_parameters_file_path.make_empty
			create leave_wall_nlsc_parameters_file_path.make_empty
			create go_to_goal_pose_controller_parameters_file_path.make_empty
			create follow_wall_pose_controller_parameters_file_path.make_empty
			create leave_wall_pose_controller_parameters_file_path.make_empty
			create wall_following_parameters_file_path.make_empty
			create range_sensors_parameters_file_path.make_empty
		end

	make_with_attributes (a_ros_path, a_goal_path, a_go_to_goal_pid_path, a_follow_wall_pid_path, a_leave_wall_pid_path,
							a_go_to_goal_nlsc_path, a_follow_wall_nlsc_path, a_leave_wall_nlsc_path, a_go_to_goal_pose_controller_path, a_follow_wall_pose_controller_path, a_leave_wall_pose_controller_path, a_wall_follow_path, a_range_sensors_path: STRING)
			-- Create files parameters object with attributes.
		do
			ros_topics_file_path := a_ros_path
			goal_parameters_file_path := a_goal_path
			go_to_goal_pid_parameters_file_path := a_go_to_goal_pid_path
			follow_wall_pid_parameters_file_path := a_follow_wall_pid_path
			leave_wall_pid_parameters_file_path := a_leave_wall_pid_path
			go_to_goal_nlsc_parameters_file_path := a_go_to_goal_nlsc_path
			follow_wall_nlsc_parameters_file_path := a_follow_wall_nlsc_path
			leave_wall_nlsc_parameters_file_path := a_leave_wall_nlsc_path
			go_to_goal_pose_controller_parameters_file_path := a_go_to_goal_pose_controller_path
			follow_wall_pose_controller_parameters_file_path := a_follow_wall_pose_controller_path
			leave_wall_pose_controller_parameters_file_path := a_leave_wall_pose_controller_path
			wall_following_parameters_file_path := a_wall_follow_path
			range_sensors_parameters_file_path := a_range_sensors_path
		end

	make_from_separate (other: separate FILES_PARAMETERS)
			-- Create files parameters object from separate object.
		do
			make_with_attributes(create {STRING}.make_from_separate (other.ros_topics_file_path),
								 create {STRING}.make_from_separate (other.goal_parameters_file_path),
								 create {STRING}.make_from_separate (other.go_to_goal_pid_parameters_file_path),
								 create {STRING}.make_from_separate (other.follow_wall_pid_parameters_file_path),
								 create {STRING}.make_from_separate (other.leave_wall_pid_parameters_file_path),
								 create {STRING}.make_from_separate (other.go_to_goal_nlsc_parameters_file_path),
								 create {STRING}.make_from_separate (other.follow_wall_nlsc_parameters_file_path),
								 create {STRING}.make_from_separate (other.leave_wall_nlsc_parameters_file_path),
								 create {STRING}.make_from_separate (other.go_to_goal_pose_controller_parameters_file_path),
								 create {STRING}.make_from_separate (other.follow_wall_pose_controller_parameters_file_path),
								 create {STRING}.make_from_separate (other.leave_wall_pose_controller_parameters_file_path),
								 create {STRING}.make_from_separate (other.wall_following_parameters_file_path),
								 create {STRING}.make_from_separate (other.range_sensors_parameters_file_path))
		end

feature -- Access

	ros_topics_file_path: STRING
			-- ROS topics file path.

	goal_parameters_file_path: STRING
			-- Goal parameters file path.

	go_to_goal_pid_parameters_file_path: STRING
			-- Go to goal state PID parameters file path.

	go_to_goal_nlsc_parameters_file_path: STRING
			-- Go to goal state non-linear speed controller parameters file path.

	go_to_goal_pose_controller_parameters_file_path: STRING
			-- Go to goal state pose controller parameters file path.

	follow_wall_pid_parameters_file_path: STRING
			-- Follow wall state PID parameters file path.

	follow_wall_nlsc_parameters_file_path: STRING
			-- Follow wall state non-linear speed controller parameters file path.

	follow_wall_pose_controller_parameters_file_path: STRING
			-- Follow wall state pose controller parameters file path.

	leave_wall_pid_parameters_file_path: STRING
			-- Leave wall state PID parameters file path.

	leave_wall_nlsc_parameters_file_path: STRING
			-- Leave wall state non-linear speed controller parameters file path.

	leave_wall_pose_controller_parameters_file_path: STRING
			-- Leave wall state pose controller parameters file path.

	wall_following_parameters_file_path: STRING
			-- Wall following parameters file path.

	range_sensors_parameters_file_path: STRING
			-- Range sensors parameters file path.

	set_goal_parameters_file_path (files_goal_parameters_file_path: STRING)
			-- Setter for `goal_parameters_file_path'.
		do
			goal_parameters_file_path.copy (files_goal_parameters_file_path)
		end

	set_go_to_goal_pid_parameters_file_path (files_go_to_goal_pid_parameters_file_path: STRING)
			-- Setter for `go_to_goal_pid_parameters_file_path'.
		do
			go_to_goal_pid_parameters_file_path.copy (files_go_to_goal_pid_parameters_file_path)
		end

	set_go_to_goal_nlsc_parameters_file_path (files_go_to_goal_nlsc_parameters_file_path: STRING)
			-- Setter for `go_to_goal_nlsc_parameters_file_path'.
		do
			go_to_goal_nlsc_parameters_file_path.copy (files_go_to_goal_nlsc_parameters_file_path)
		end

	set_go_to_goal_pose_controller_parameters_file_path (files_go_to_goal_pose_controller_parameters_file_path: STRING)
			-- Setter for `go_to_goal_pose_controller_parameters_file_path'.
		do
			go_to_goal_pose_controller_parameters_file_path.copy (files_go_to_goal_pose_controller_parameters_file_path)
		end

	set_follow_wall_pid_parameters_file_path (files_follow_wall_pid_parameters_file_path: STRING)
			-- Setter for `follow_wall_pid_parameters_file_path'.
		do
			follow_wall_pid_parameters_file_path.copy (files_follow_wall_pid_parameters_file_path)
		end

	set_follow_wall_nlsc_parameters_file_path (files_follow_wall_nlsc_parameters_file_path: STRING)
			-- Setter for `follow_wall_nlsc_parameters_file_path'.
		do
			follow_wall_nlsc_parameters_file_path.copy (files_follow_wall_nlsc_parameters_file_path)
		end

	set_follow_wall_pose_controller_parameters_file_path (files_follow_wall_pose_controller_parameters_file_path: STRING)
			-- Setter for `follow_wall_pose_controller_parameters_file_path'.
		do
			follow_wall_pose_controller_parameters_file_path.copy (files_follow_wall_pose_controller_parameters_file_path)
		end

	set_leave_wall_pid_parameters_file_path (files_leave_wall_pid_parameters_file_path: STRING)
			-- Setter for `leave_wall_pid_parameters_file_path'.
		do
			leave_wall_pid_parameters_file_path.copy (files_leave_wall_pid_parameters_file_path)
		end

	set_leave_wall_nlsc_parameters_file_path (files_leave_wall_nlsc_parameters_file_path: STRING)
			-- Setter for `leave_wall_nlsc_parameters_file_path'.
		do
			leave_wall_nlsc_parameters_file_path.copy (files_leave_wall_nlsc_parameters_file_path)
		end

	set_leave_wall_pose_controller_parameters_file_path (files_leave_wall_pose_controller_parameters_file_path: STRING)
			-- Setter for `leave_wall_pose_controller_parameters_file_path'.
		do
			leave_wall_pose_controller_parameters_file_path.copy (files_leave_wall_pose_controller_parameters_file_path)
		end

	set_wall_following_parameters_file_path (files_wall_following_parameters_file_path: STRING)
			-- Setter for `wall_following_parameters_file_path'.
		do
			wall_following_parameters_file_path.copy (files_wall_following_parameters_file_path)
		end

	set_range_sensors_parameters_file_path (files_range_sensors_parameters_file_path: STRING)
			-- Setter for `range_sensors_parameters_file_path'.
		do
			range_sensors_parameters_file_path.copy (files_range_sensors_parameters_file_path)
		end

	set_ros_topics_file_path (files_range_sensors_parameters_file_path: STRING)
			-- Setter for `range_sensors_parameters_file_path'.
		do
			ros_topics_file_path.copy (files_range_sensors_parameters_file_path)
		end
end
