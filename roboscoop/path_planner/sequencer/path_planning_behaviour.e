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

	make_with_attributes (param_bag: separate PATH_PLANNER_PARAMETERS_BAG)
			-- Create Current with signaler.
		do
			create map_signaler.make_with_topic (param_bag.path_planner_topics.map)
			create path_publisher.make_with_topic (param_bag.path_planner_topics.path)
			create stop_signaler.make

			create path_planning_signaler.make_with_attributes (param_bag.path_planner_parameters.edge_cost, param_bag.path_planner_parameters.heuristic_cost, param_bag.path_planner_parameters.bfs, param_bag.path_planner_parameters.dfs, param_bag.path_planner_parameters.dijkstra, param_bag.path_planner_parameters.frame_id)
			create map_parameters_signaler.make_with_attributes (param_bag.map_parameters.blocking, param_bag.map_parameters.inflation, param_bag.map_parameters.connectivity_strategy)
		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate PATH_PLANNING_CONTROLLER
		do
			create a.make (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a, path_planning_signaler, map_parameters_signaler)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

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

	sep_start (a: separate PATH_PLANNING_CONTROLLER; path_plan_sig: separate PATH_PLANNING_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Start controllers asynchronously.
		do
			a.search (map_signaler, map_params_sig, path_planning_signaler, path_publisher)
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

end
