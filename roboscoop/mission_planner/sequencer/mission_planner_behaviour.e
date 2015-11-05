note
	description: "Execute asyncrhonously the mission."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes ()
			-- Create `Current' with attributes.
		do
			create start_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.path_planner_start)
			create goal_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.path_planner_goal)
			create target_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.target)
			create stop_signaler.make
		end

feature {ANY} -- Access

	start
			-- Start the behaviour.
		local
			--a: separate PATH_PLANNING_CONTROLLER
		do
			--publish_way_points (map_signaler, path_planning_signaler, node_publisher)
			--create a.make (stop_signaler)
			--sep_stop (stop_signaler, False)
			--sep_start (a, path_planning_signaler, map_parameters_signaler)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation


	start_publisher: separate POINT_PUBLISHER
			-- Publisher of start point.

	goal_publisher: separate POINT_PUBLISHER
			-- Publisher of goal point.

	target_publisher: separate POINT_PUBLISHER
			-- Publisher of current target point.

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start ()
			-- Start controllers asynchronously.
		do
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end
