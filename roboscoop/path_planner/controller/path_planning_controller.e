note
	description: "Controller for the path-planning algorithm procedure."
	author: "Sebastian Curi"
	date: "30.10.2015"

class
	PATH_PLANNING_CONTROLLER

inherit

	CANCELLABLE_CONTROL_LOOP

create
	make

feature {NONE} -- Initialization

	make (s_sig: separate STOP_SIGNALER)
			-- Create `Current' and assign given attributes.
		do
			stop_signaler := s_sig
		end

feature {PATH_PLANNING_BEHAVIOUR} -- Execute algorithm

	update_map (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- update map
		require
			map.state.header.timestamp > map_params_sig.timestamp
		do
			map.inflate (map_params_sig.inflation)
			create grid_wrapper.make_with_connectivity (map, map_params_sig.connectivity_strategy, map_params_sig.block_width, map_params_sig.block_height)
			map_params_sig.set_timestamp(map.state.header.timestamp)
		end

	search (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER; path_publisher: separate PATH_PUBLISHER)
			-- Apply search strategy
		require
			map.state.info.resolution > 0
		local
			search_algorithm: LABEL_CORRECTING_GRAPH_SEARCH
			start: SPATIAL_GRAPH_NODE
			goal: SPATIAL_GRAPH_NODE
			open_set: DISPENSER [LABELED_NODE]
			idx: INTEGER
			path: ARRAYED_STACK [SPATIAL_GRAPH_NODE]
		do
			open_set := create {ARRAYED_QUEUE [LABELED_NODE]}.make (0)

			if path_planning_sig.bfs then
				open_set := create {LINKED_QUEUE [LABELED_NODE]}.make
			elseif path_planning_sig.dfs then
				open_set := create {LINKED_STACK [LABELED_NODE]}.make
			elseif path_planning_sig.dijkstra then
				open_set := create {HEAP_PRIORITY_QUEUE [LABELED_NODE]}.make (grid_wrapper.grid.nodes.count)
			end

				-- Find nearest index of start and goal nodes
--			from
--				idx := 1
--			until
--				idx > path_planning_sig.way_points.count - 1
--			loop
--				create search_algorithm.make_with_graph (grid_wrapper.grid)
--				start := grid_wrapper.grid.node_at (((path_planning_sig.way_points.at (idx).position.x - map.state.info.origin.position.x) / map.state.info.resolution).ceiling, ((path_planning_sig.way_points.at (idx).position.y - map.state.info.origin.position.y) / map.state.info.resolution).ceiling, 1)
--				goal := grid_wrapper.grid.node_at (((path_planning_sig.way_points.at (idx + 1).position.x - map.state.info.origin.position.x) / map.state.info.resolution).ceiling, ((path_planning_sig.way_points.at (idx + 1).position.y - map.state.info.origin.position.y) / map.state.info.resolution).ceiling, 1)
--				path := search_algorithm.search (start, goal, path_planning_sig.edge_cost_strategy, path_planning_sig.heuristic_strategy, open_set)
--				path_publisher.publish_path (path, path_planning_sig.frame)
--				idx := idx + 1
--			end

			create search_algorithm.make_with_graph (grid_wrapper.grid)

			start := grid_wrapper.grid.node_at (((path_planning_sig.start_point.x - map.state.info.origin.position.x) / map.state.info.resolution).ceiling, ((path_planning_sig.start_point.y - map.state.info.origin.position.y) / map.state.info.resolution).ceiling, 1)
			goal := grid_wrapper.grid.node_at (((path_planning_sig.goal_point.x - map.state.info.origin.position.x) / map.state.info.resolution).ceiling, ((path_planning_sig.goal_point.y - map.state.info.origin.position.y) / map.state.info.resolution).ceiling, 1)
			path := search_algorithm.search (start, goal, path_planning_sig.edge_cost_strategy, path_planning_sig.heuristic_strategy, open_set)
			path_publisher.publish_path (path, path_planning_sig.frame)

			separate stop_signaler as s_sig do
				s_sig.set_stop_requested (True)
			end
		end

feature {NONE} -- Implementation

	grid_wrapper: GRID_FROM_MAP
end
