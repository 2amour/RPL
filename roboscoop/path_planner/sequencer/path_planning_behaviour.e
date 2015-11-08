note
	description: "Execute asynchronously the path planning algorithm."
	author: "Sebastian Curi"
	date: "29.10.2015"

class
	PATH_PLANNING_BEHAVIOUR

inherit

	BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (param_bag: PATH_PLANNER_PARAMETERS_BAG)
			-- Create Current with signaler.
		do
			create start_signaler.make_with_topic (param_bag.path_planner_topics.start)
			create goal_signaler.make_with_topic (param_bag.path_planner_topics.goal)
			create map_signaler.make_with_topic (param_bag.path_planner_topics.map)
			create path_publisher.make_with_topic (param_bag.path_planner_topics.path)
			create stop_signaler.make

			create path_planning_signaler.make_with_attributes (param_bag.path_planner_parameters.search_strategy, param_bag.path_planner_parameters.edge_cost, param_bag.path_planner_parameters.heuristic_cost, param_bag.path_planner_parameters.frame_id)
			create map_parameters_signaler.make_with_attributes (param_bag.map_parameters.block_width, param_bag.map_parameters.block_height, param_bag.map_parameters.inflation, param_bag.map_parameters.connectivity_strategy)
		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a, b, c, d, e, f: separate PATH_PLANNING_CONTROLLER
		do
			create a.make (stop_signaler)
			create b.make (stop_signaler)
			create c.make (stop_signaler)
			create d.make (stop_signaler)
			create e.make (stop_signaler)
			--create f.make (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a, b, c, d, e)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	start_signaler: separate POINT_SIGNALER
			-- Signaler of the start position.

	goal_signaler: separate POINT_SIGNALER
			-- Signaler of the goal position.

	path_planning_signaler: separate PATH_PLANNING_SIGNALER
			-- Signaler with path planning algorithm states.

	map_parameters_signaler: separate MAP_PARAMETERS_SIGNALER
			-- Signaler with input map parameters.

	map_signaler: separate OCCUPANCY_GRID_SIGNALER
			-- Signaler with map data.

	path_publisher: separate PATH_PUBLISHER
			-- Publisher of resultig path.

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start (a, b, c, d, e: separate PATH_PLANNING_CONTROLLER)
			-- Start controllers asynchronously.
		do
			a.repeat_until_stop_requested (agent a.search (map_signaler, map_parameters_signaler, path_planning_signaler, path_publisher))
			b.repeat_until_stop_requested (agent b.update_search (map_signaler, map_parameters_signaler, path_planning_signaler, path_publisher))

			c.repeat_until_stop_requested (agent c.update_start_point (start_signaler, path_planning_signaler))
			d.repeat_until_stop_requested (agent d.update_goal_point (goal_signaler, path_planning_signaler))
			e.repeat_until_stop_requested (agent e.update_map (map_signaler, map_parameters_signaler))
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end
