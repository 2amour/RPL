note
	description: "Execute asyncrhonously the mission."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (parameters_bag: MISSION_PLANNER_PARAMETERS_BAG)
			-- Create `Current' with attributes.
		do
			create mission_signaler.make_with_attributes (parameters_bag.mission_planner_parameters.frame, parameters_bag.mission_planner_parameters.way_points, parameters_bag.mission_planner_parameters.way_point_threshold)

			create obstacle_signaler.make_with_topic (parameters_bag.mission_planner_topics.sensed_obstacle)
			create map_signaler.make_with_topic (parameters_bag.mission_planner_topics.map)
			create odometry_signaler.make_with_topic (parameters_bag.mission_planner_topics.odometry)
			create path_signaler.make_with_topic (parameters_bag.mission_planner_topics.path)
			create object_recognition_signaler.make_with_topic (parameters_bag.mission_planner_topics.object_recognition_signaler)

			create localization_publisher.make_with_topic (parameters_bag.mission_planner_topics.localization_request)

			create start_publisher.make_with_topic (parameters_bag.mission_planner_topics.path_planner_start)
			create goal_publisher.make_with_topic (parameters_bag.mission_planner_topics.path_planner_goal)
			create target_publisher.make_with_topic (parameters_bag.mission_planner_topics.target)
			create map_publisher.make_with_topic (parameters_bag.mission_planner_topics.planner_map)
			set_map_frame (map_publisher, parameters_bag.mission_planner_topics.planner_map_frame)
			create object_recognition_publisher.make_with_topic (parameters_bag.mission_planner_topics.object_recognition_request)
			create localization_signaler.make_with_topic (parameters_bag.mission_planner_topics.localization_signaler)
			create stop_signaler.make
		end

feature {ANY} -- Access

	start
			-- Start the behaviour.
		local
			a, b, c, d, e, f: separate MISSION_PLANNER_CONTROLLER
		do
			create a.make (stop_signaler)
			create b.make (stop_signaler)
			create c.make (stop_signaler)
			create d.make (stop_signaler)
			create e.make (stop_signaler)
			create f.make (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a, b, c, d, e, f)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	-- MISSION PLANER STATE SIGNALER
	mission_signaler: separate MISSION_PLANNER_SIGNALER
			-- Mission signaler

	-- ROBOT CONTROLLER RELATED PUBLISHERS
	obstacle_signaler: separate POINT_SIGNALER
			-- Signaler with detected obstacles.

	odometry_signaler: separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	target_publisher: separate POSE_PUBLISHER
			-- Publisher of current target point.

	-- PATH PLANNING RELATED PUBLISHERS
	map_publisher: separate OCCUPANCY_GRID_PUBLISHER
			-- Signaler with map data.

	map_signaler: separate OCCUPANCY_GRID_SIGNALER
			-- Signaler with map data.

	path_signaler: separate PATH_SIGNALER_WITH_FLAG
			-- Current state of the path.

	start_publisher: separate POSE_PUBLISHER
			-- Publisher of start point.

	goal_publisher: separate POSE_PUBLISHER
			-- Publisher of goal point.

	-- OBJECT RECOGNITION RELATED PUBLISHERS
	object_recognition_publisher: separate EMPTY_PUBLISHER
			-- Publisher to request object recognition procedure.

	object_recognition_signaler: separate EMPTY_SIGNALER
			-- Signaler to handle the object recognition module timing.

	-- LOCALIZATION RELATED PUBLISHERS

	localization_publisher: separate BOOLEAN_PUBLISHER
			-- Publisher to request object recognition procedure.

	localization_signaler: separate BOOLEAN_SIGNALER
			-- Signaler to handle the object recognition module timing.


	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start (a, b, c, d, e, f: separate MISSION_PLANNER_CONTROLLER)
			-- Start controllers asynchronously.
		do
			a.repeat_until_stop_requested (agent a.update_target(odometry_signaler, localization_signaler, mission_signaler, target_publisher, object_recognition_signaler, stop_signaler))
			b.repeat_until_stop_requested (agent b.request_path(mission_signaler, obstacle_signaler, start_publisher, goal_publisher, stop_signaler))
			c.repeat_until_stop_requested (agent c.update_path(mission_signaler, path_signaler, stop_signaler))
			d.repeat_until_stop_requested (agent d.update_map (obstacle_signaler, mission_signaler, map_signaler, map_publisher, stop_signaler))
			e.repeat_until_stop_requested (agent e.request_recognition (object_recognition_publisher, object_recognition_signaler, mission_signaler, odometry_signaler, stop_signaler))
			f.repeat_until_stop_requested (agent f.request_localization (localization_publisher, mission_signaler, stop_signaler))
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

	set_map_frame(map_pub: separate OCCUPANCY_GRID_PUBLISHER; a_frame: separate STRING)
			-- Set the frame of the occupancy grid publisher
		do
			map_pub.set_frame (a_frame)
		end
end
