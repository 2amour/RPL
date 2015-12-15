note
	description: "Robot controller ROS topics."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	ROBOT_CONTROLLER_TOPIC_PARAMETERS

inherit

	PARAMETERS

create
	make_default, make_from_separate

feature {NONE} -- Initialize

	make_default
			-- Make default
		do
			name := "robot_controller"
			path := "/robot_controller/path"
			pose := "/robot_controller/pose"

			mission_odometry := "/robot_controller/odometry"
			sensed_obstacles := "/robot_controller/sensed_obstacles"

			goal := "/robot_controller/target"

			circular_leds_topic := "/aseba/events/circle_leds_cmd"
			visualization_marker := "/visualization_marker"

		end

	make_from_separate (other: separate ROBOT_CONTROLLER_TOPIC_PARAMETERS)
			-- Make from separate.
		do
			create name.make_from_separate (other.name)
			create path.make_from_separate (other.path)
			create pose.make_from_separate (other.pose)

			create mission_odometry.make_from_separate (other.mission_odometry)
			create sensed_obstacles.make_from_separate (other.sensed_obstacles)

			create goal.make_from_separate (other.goal)

			create circular_leds_topic.make_from_separate (other.circular_leds_topic)
			create visualization_marker.make_from_separate (other.visualization_marker)

		end

feature -- Access

		name: STRING
				-- Node name.

		set_name (a_topic: separate STRING)
				-- Set node name.
			do
				create name.make_from_separate (a_topic)
			end

		path: STRING
			-- Path topic .

		set_path (a_topic: separate STRING)
				-- Set path topic.
			do
				create path.make_from_separate (a_topic)
			end

		pose: STRING
			-- Pose topic .

		set_pose (a_topic: separate STRING)
				-- Set pose topic.
			do
				create pose.make_from_separate (a_topic)
			end

		mission_odometry: STRING
				-- Mission odometry topic .

		set_mission_odometry (a_topic: separate STRING)
				-- Set mission odometry topic.
			do
				create mission_odometry.make_from_separate (a_topic)
			end

		sensed_obstacles: STRING
			-- Sensed obstacles topic .

		set_sensed_obstacles (a_topic: separate STRING)
				-- Set sensed obstacles topic.
			do
				create sensed_obstacles.make_from_separate (a_topic)
			end

		goal: STRING
			-- goal odometry topic .

		set_goal (a_topic: separate STRING)
				-- Set mission odometry topic.
			do
				create goal.make_from_separate (a_topic)
			end

		circular_leds_topic: STRING
			-- Circular_leds topic .

		set_circular_leds (a_topic: separate STRING)
				-- Set circular leds topic.
			do
				create circular_leds_topic.make_from_separate (a_topic)
			end

		visualization_marker: STRING
			-- Visualization marker topic .

		set_visualization_marker (a_topic: separate STRING)
				-- Set visualization marker topic.
			do
				create visualization_marker.make_from_separate (a_topic)
			end

end
