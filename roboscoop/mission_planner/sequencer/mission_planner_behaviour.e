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
			create mission_signaler.make_with_attributes (parameters_bag.mission_planner_parameters.way_points, parameters_bag.mission_planner_parameters.way_point_threshold)
			create odometry_signaler.make_with_topic (parameters_bag.mission_planner_topics.odometry)
			create path_signaler.make_with_topic (parameters_bag.mission_planner_topics.path)

			create start_publisher.make_with_topic (parameters_bag.mission_planner_topics.path_planner_start)
			create goal_publisher.make_with_topic (parameters_bag.mission_planner_topics.path_planner_goal)
			create target_publisher.make_with_topic (parameters_bag.mission_planner_topics.target)

			--create target_path_publisher.make_with_topic("/robot_controller/path")
			create stop_signaler.make
		end

feature {ANY} -- Access

	start
			-- Start the behaviour.
		local
			a, b, c: separate MISSION_PLANNER_CONTROLLER
		do
			create a.make (stop_signaler)
			create b.make (stop_signaler)
			create c.make (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a, b, c)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	mission_signaler: separate MISSION_PLANNER_SIGNALER
			-- Mission signaler

	odometry_signaler: separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	path_signaler: separate PATH_SIGNALER_WITH_FLAG
			-- Current state of the path.

	start_publisher: separate POINT_PUBLISHER
			-- Publisher of start point.

	goal_publisher: separate POINT_PUBLISHER
			-- Publisher of goal point.

	target_publisher: separate POINT_PUBLISHER
			-- Publisher of current target point.

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start (a, b, c: separate MISSION_PLANNER_CONTROLLER)
			-- Start controllers asynchronously.
		do
			b.repeat_until_stop_requested (agent b.request_path(mission_signaler, start_publisher, goal_publisher))
			c.repeat_until_stop_requested (agent c.update_path(mission_signaler, path_signaler))
			a.repeat_until_stop_requested (agent a.update_target(odometry_signaler, mission_signaler, target_publisher))
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end
