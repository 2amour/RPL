note
	description: "State of Tangent bug."
	author: "Sebastian Curi"
	date: "18.10.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (goal_parameters: separate GOAL_PARAMETERS; pid_parameters: separate PID_PARAMETERS; wall_following_parameters: separate WALL_FOLLOWING_PARAMETERS)
			-- Initialize signaler with attributes.
		do
			create goal.make_with_coordinates (0.0, 0.0)
			create current_pose.make
			create intial_point_wall.make

			goal_threshold := goal_parameters.threshold
			timestamp := 0.0
			min_distance := {REAL_64}.max_value

			initialize_states (create {PID_PARAMETERS}.make_from_separate (pid_parameters), create {WALL_FOLLOWING_PARAMETERS}.make_from_separate (wall_following_parameters))
			set_go_to_goal
		end

	initialize_states (pid_parameters: PID_PARAMETERS; wall_following_parameters: WALL_FOLLOWING_PARAMETERS)
			-- Initialize states.
		do
			create at_goal
			create follow_wall.make_with_attributes (pid_parameters, wall_following_parameters)
			create go_to_goal.make_with_attributes (pid_parameters)
			create leave_wall.make_with_attributes (pid_parameters)
			create unreachable_goal
		end

feature -- Access

	goal: POINT_2D
			-- Goal coordinates.

	goal_threshold: REAL_64
			-- Threshold for considering when the goal is reached.

	current_pose: POSE_2D
			-- Present position and orientation.

	timestamp: REAL_64
			-- Time.

	min_distance: REAL_64
			-- Minimum recorded distance to goal.

	intial_point_wall: POINT_2D
			-- Position at the beginning of finding a wall.

	state: TANGENT_BUG_STATE
			-- State of the robot.

feature -- Status report

	is_go_to_goal: BOOLEAN
			-- Whether state is 'go_to_goal'.

	is_follow_wall: BOOLEAN
			-- Whether state is 'follow_wall_cw'.

	is_leave_wall: BOOLEAN
			-- Whether state is 'leave_wall'.

	is_at_goal: BOOLEAN
			-- Whether state is 'at_goal'.

	is_unreachable_goal: BOOLEAN
			-- Whether state is 'unreachable_goal'.

feature -- Status setting

	set_at_goal
			-- Set at goal state.
		do
			state := at_goal
			is_at_goal := True
			debug
				io.put_string ("At Goal %N")
			end
		end

	set_follow_wall_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_clockwise
			state := follow_wall
			is_follow_wall := True
			debug
				io.put_string ("Follow Wall Clockwise %N")
			end
		end

	set_follow_wall_counter_clockwise
			-- Set follow wall state.
		do
			follow_wall.set_counter_clockwise
			state := follow_wall
			is_follow_wall := True
			debug
				io.put_string ("Follow Wall Counter Clockwise%N")
			end
		end

	set_leave_wall_with_target (p: separate POINT_2D)
			-- Set to leave wall state.
		do
			leave_wall.set_target (create {POINT_2D}.make_with_coordinates (p.get_x, p.get_y))
			state := leave_wall
			is_leave_wall := True
			debug
				io.put_string ("Leave Wall %N")
			end
		end

	set_go_to_goal
			-- Set to go to goal state.
		do
			state := go_to_goal
			is_go_to_goal := True
			debug
				io.put_string ("Go to Goal %N")
			end
		end

	set_unreachable_goal
			-- Set to unreachable goal state.
		do
			state := unreachable_goal
			is_unreachable_goal := True
			debug
				io.put_string ("Unreachable Wall %N")
			end
		end

feature -- Element change

	set_goal (x, y: REAL_64)
			-- Setter for `goal'.
		do
			create goal.make_with_coordinates (x, y)
			min_distance := {REAL_64}.max_value
		end

	set_min_distance (d: REAL_64)
			-- Set minimum distance to goal.
		require
			valid_d: d > 0
		do
			min_distance := d
		ensure
			d_set: d = min_distance
		end

	set_pose (pose: separate POSE_2D)
			-- Set new pose.
		do
			current_pose := create {POSE_2D}.make_from_separate (pose)
		ensure
			pose_set: current_pose = pose
		end

	set_timestamp (t: REAL_64)
			-- Set time
		do
			timestamp := t
		ensure
			time_set: timestamp = t
		end

feature {NONE} -- Implementation

	at_goal: AT_GOAL
			-- State "at_goal".

	follow_wall: FOLLOW_WALL
			-- State "follow_wall".

	go_to_goal: GO_TO_GOAL
			-- State "go_to_goal".

	leave_wall: LEAVE_WALL
			-- State "leave_wall".

	unreachable_goal: UNREACHABLE_GOAL
			-- State "unreachable_goal".
end
