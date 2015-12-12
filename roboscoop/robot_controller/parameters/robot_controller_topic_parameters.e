note
	description: "Robot controller ROS topics."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	ROBOT_CONTROLLER_TOPIC_PARAMETERS

inherit

	TOPIC_PARAMETERS

create
	make_default, make_with_attributes

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

	make_with_attributes (a_name, a_path, a_pose, a_odometry, a_obstacle, a_goal, a_circular_led, a_viz_mark: STRING)
			-- Make with attributes
		do
			name := a_name
			path := a_path
			pose := a_pose

			mission_odometry := a_odometry
			sensed_obstacles := a_obstacle
			goal := a_goal


			circular_leds_topic := a_circular_led
			visualization_marker := a_viz_mark
		end

	make_from_separate (other: separate like Current)
			-- Create object as a copy of a separate object.
		do
			make_with_attributes (create {STRING}.make_from_separate (other.name),
								  create {STRING}.make_from_separate (other.path),
								  create {STRING}.make_from_separate (other.pose),
								  create {STRING}.make_from_separate (other.mission_odometry),
								  create {STRING}.make_from_separate (other.sensed_obstacles),
								  create {STRING}.make_from_separate (other.goal),
								  create {STRING}.make_from_separate (other.circular_leds_topic),
								  create {STRING}.make_from_separate (other.visualization_marker))
		end

feature {ANY} -- Access

		name: STRING_8
		path: STRING_8
		pose: STRING_8

		mission_odometry: STRING_8
		sensed_obstacles: STRING_8

		goal: STRING_8

		circular_leds_topic: STRING_8
		visualization_marker: STRING_8

end
