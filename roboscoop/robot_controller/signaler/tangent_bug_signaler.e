note
	description: "State of Tangent bug."
	author: "Sebastian Curi"
	date: "18.10.15"

class
	TANGENT_BUG_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (parameters_bag: separate TANGENT_BUG_PARAMETERS_BAG)
			-- Initialize signaler with attributes.
		local
			goal_parameters: GOAL_PARAMETERS
			go_to_goal_pose_controller_parameters: POSE_CONTROLLER_PARAMETERS
		do
			create goal_parameters.make_from_separate (parameters_bag.goal_parameters)
			create go_to_goal_pose_controller_parameters.make_from_separate (parameters_bag.go_to_goal_pose_controller_parameters)

			create goal.make_with_coordinates (goal_parameters.x, goal_parameters.y, goal_parameters.orientation)
			create current_pose.make
			create intial_point_wall.make

			reached_pose_position_threshold := go_to_goal_pose_controller_parameters.reached_point_threshold
			reached_pose_orientation_threshold := go_to_goal_pose_controller_parameters.reached_orientation_threshold
			timestamp := 0.0
			min_distance := {REAL_64}.max_value

			initialize_states (parameters_bag)
			set_go_to_goal
		end

	initialize_states (parameters_bag: separate TANGENT_BUG_PARAMETERS_BAG)
			-- Initialize states.
		do
			create go_to_goal.make_with_attributes (parameters_bag.go_to_goal_pose_controller_parameters)
			create follow_wall.make_with_attributes (parameters_bag.follow_wall_pose_controller_parameters, parameters_bag.wall_following_parameters)
			create leave_wall.make_with_attributes (parameters_bag.leave_wall_pose_controller_parameters)
			create at_goal
			create unreachable_goal
		end

feature -- Access

	goal: POSE_2D
			-- Goal coordinates.

	reached_pose_position_threshold: REAL_64
			-- Threshold for considering when the pose position is reached.

	reached_pose_orientation_threshold: REAL_64
			-- Threshold for considering when the pose orientation is reached.

	current_pose: POSE_2D
			-- Present position and orientation.

	timestamp: REAL_64
			-- Time.

	min_distance: REAL_64
			-- Minimum recorded distance to goal.

	initial_orientation: REAL_64
			-- Orientation at the beginning of finding a wall.

	has_turned_back: BOOLEAN
			-- Whether the robot has turned more than ninety degrees from `initial_orientation'.

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
				io.put_string ("Follow Wall Counter Clockwise %N")
			end
		end

	set_leave_wall_with_target (p: separate POINT_2D)
			-- Set to leave wall state.
		local
			heading: REAL_64
			math: TRIGONOMETRY_MATH
		do
			create math
			heading := math.atan2 (p.get_y - current_pose.get_position.get_y, p.get_x - current_pose.get_position.get_x) - current_pose.get_orientation
			heading := math.atan2 (math.sine (heading), math.cosine (heading))
			leave_wall.set_safe_sensed_pose (create {POSE_2D}.make_with_pose (p, current_pose.get_orientation + heading))
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

	set_goal (a_goal: separate POSE_2D)
			-- Setter for `goal'.
		do
			create goal.make_from_separate (a_goal)
			min_distance := goal.get_position.get_euclidean_distance (current_pose.get_position)
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
			-- Set time.
		do
			timestamp := t
		ensure
			time_set: timestamp = t
		end

	set_intial_point_wall (point: separate POINT_2D)
			-- Set inital point wall.
		do
			intial_point_wall.set_coordinates (point.get_x, point.get_y)
		end

	set_initial_orientation (orientation: REAL_64)
			-- Set initial orientation.
		do
			initial_orientation := orientation
		end

	set_has_turned_back (bool: BOOLEAN)
			-- Set turned back.
		do
			has_turned_back := bool
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
