note
	description: "Execute asyncrhonously the mission."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (mission_sig: separate MISSION_PLANNER_SIGNALER)
			-- Create `Current' with attributes.
		do
			mission_signaler := mission_sig
			create odometry_signaler.make_with_topic ({MISSION_PLANNER_TOPICS}.odometry)
			create path_signaler.make_with_topic ({MISSION_PLANNER_TOPICS}.path)

			create start_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.path_planner_start)
			create goal_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.path_planner_goal)
			create target_publisher.make_with_topic ({MISSION_PLANNER_TOPICS}.target)

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

	path_signaler: separate PATH_SIGNALER
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
			a.repeat_until_stop_requested (agent a.update_target(odometry_signaler, mission_signaler, target_publisher))
			b.repeat_until_stop_requested (agent b.update_path(path_signaler, mission_signaler))
			c.repeat_until_stop_requested (agent c.request_path(mission_signaler, start_publisher, goal_publisher))
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end
