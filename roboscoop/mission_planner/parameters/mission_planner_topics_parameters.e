note
	description: "Mission Planner Topics."
	author: "Sebastian Curi"
	date: "5.11.2015"

class
	MISSION_PLANNER_TOPICS_PARAMETERS

inherit

	TOPIC_PARAMETERS

create
	make_default

feature {NONE} -- Initialize

	make_default
			-- Make with default values.
		do
			node_name := "mission_planner_node"
			map := "/map"

			target := "/robot_controller/goal"
			odometry := "/robot_controller/odometry"
			sensed_obstacle := "/robot_controller/obstacles"

			path := "/path_planner/path"
			path_planner_start := "/path_planner/start"
			path_planner_goal := "/path_planner/goal"
			planner_map := "/path_planner/map"
			planner_map_frame := "/map"
			object_recognition_request := "/object_recognition/request"
			object_recognition_signaler := "/object_recognition/image_recieved"

			localization_request := "/localization/request"
			localization_signaler := "/localization/is_localized"
		end

feature {ANY} -- Constants

	node_name: STRING
			-- Name of node.

	set_node_name(a_node_name: separate STRING)
			-- Set the node name.
		do
			node_name := create {STRING}.make_from_separate (a_node_name)
		end

	map: STRING
			-- Map to be published.

	set_map(a_map: separate STRING)
			-- Set the map name.
		do
			map := create {STRING}.make_from_separate (a_map)
		end

	target: STRING
			-- Target goal for the driver.

	set_target(a_target: separate STRING)
			-- Set the target topic.
		do
			target := create {STRING}.make_from_separate (a_target)
		end

	odometry: STRING
			-- Odometry topic.

	set_odometry(a_odometry: separate STRING)
			-- Set the odometry topic.
		do
			odometry := create {STRING}.make_from_separate (a_odometry)
		end

	sensed_obstacle: STRING
			-- Sensed obstacles topic.

	set_sensed_obstacle(a_sensed_obstacle: separate STRING)
			-- Set the sensed_obstacle topic.
		do
			sensed_obstacle := create {STRING}.make_from_separate (a_sensed_obstacle)
		end

	path: STRING
			-- Topic where path will be read.

	set_path(a_path: separate STRING)
			-- Set the path topic.
		do
			path := create {STRING}.make_from_separate (a_path)
		end

	path_planner_start: STRING
			-- Start position for path_planner node.

	set_path_planner_start(a_path_planner_start: separate STRING)
			-- Set the path_planner_start topic.
		do
			path_planner_start := create {STRING}.make_from_separate (a_path_planner_start)
		end

	path_planner_goal: STRING
			-- Goal position for path_planner node.

	set_path_planner_goal(a_path_planner_goal: separate STRING)
			-- Set the path_planner_goal topic.
		do
			path_planner_goal := create {STRING}.make_from_separate (a_path_planner_goal)
		end

	planner_map: STRING
			-- Map topic for path_planner node.

	set_planner_map(a_planner_map: separate STRING)
			-- Set the planner_map topic.
		do
			planner_map := create {STRING}.make_from_separate (a_planner_map)
		end

	planner_map_frame: STRING
			-- Frame of the map for the path_planner node.

	set_planner_map_frame(a_frame: separate STRING)
			-- Set frame name for map of the path_planner node.
		do
			planner_map_frame := create {STRING}.make_from_separate (a_frame)
		end

	object_recognition_request: STRING
			-- Topic for requesting the object_recognition module.

	set_object_recognition_request(a_object_recognition_request: separate STRING)
			-- Set the object recognition request topic.
		do
			object_recognition_request := create {STRING}.make_from_separate (a_object_recognition_request)
		end

	object_recognition_signaler: STRING
			-- Topic for signaling object_recognition completion.

	set_object_recognition_signaler(a_object_recognition_signaler: separate STRING)
			-- Set the object recognition signaler topic.
		do
			object_recognition_signaler := create {STRING}.make_from_separate (a_object_recognition_signaler)
		end


	localization_request: STRING
			-- Topic to request localization algorithm.

	set_localization_request(a_localization_request: separate STRING)
			-- Set the request localization topic.
		do
			localization_request := create {STRING}.make_from_separate (a_localization_request)
		end

	localization_signaler: STRING
			-- Topic to check if robot is localized.

	set_localization_signaler(a_localization_signaler: separate STRING)
			-- Set the localization status topic.
		do
			localization_signaler := create {STRING}.make_from_separate (a_localization_signaler)
		end

end
