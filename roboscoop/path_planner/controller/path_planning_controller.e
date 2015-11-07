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
			search_algorithm := create {A_STAR}.make_default
			stop_signaler := s_sig
		end

feature {ANY} -- Access

	is_grid_attached: BOOLEAN
			-- Check if grid is attached.
		do
			Result := attached grid_wrapper
		end

feature {PATH_PLANNING_BEHAVIOUR} -- Execute algorithm

	update_start_point (start_sig: separate POINT_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER)
			-- Update start point.
		require
			start_sig.new_point
		do
			path_planning_sig.set_start (start_sig.point)
			start_sig.set_new_point(False)
		end

	update_goal_point (goal_sig: separate POINT_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER)
			-- Update goal point.
		require
			goal_sig.new_point
		do
			path_planning_sig.set_goal (goal_sig.point)
			goal_sig.set_new_point(False)
		end

	update_map (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Update map.
		require
			map.state.header.timestamp > map_params_sig.timestamp
		do
			map.inflate (map_params_sig.inflation)
			create grid_wrapper.make_with_connectivity (map, map_params_sig.connectivity_strategy, map_params_sig.block_width, map_params_sig.block_height)
			map_params_sig.set_timestamp(map.state.header.timestamp)
			map_params_sig.set_changed (True)
		end

	search (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER; path_publisher: separate PATH_PUBLISHER)
			-- Execute search algorithm.
		require
			is_grid_attached
			attached path_planning_sig.start_point
			attached path_planning_sig.goal_point
			map_params_sig.is_changed or path_planning_sig.changed_start
		local
			start_point: POINT
			goal_point: POINT
			start_node: SPATIAL_GRAPH_NODE
			goal_node: SPATIAL_GRAPH_NODE
			open_set: DISPENSER [LABELED_NODE]
			idx: INTEGER
			path: ARRAYED_STACK [SPATIAL_GRAPH_NODE]
		do
			if attached grid_wrapper as gw and
			   attached path_planning_sig.start_point as sp and
			   attached path_planning_sig.goal_point as gp then

				create start_point.make_from_separate (sp)
				create goal_point.make_from_separate (gp)
				search_algorithm := create {A_STAR}.make_with_graph (gw.grid)

				start_node := gw.grid.node_at (((start_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((start_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)
				goal_node := gw.grid.node_at (((goal_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((goal_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)
				path := search_algorithm.search (start_node, goal_node, path_planning_sig.edge_cost_strategy, path_planning_sig.heuristic_strategy)
				path_publisher.publish_path (path, path_planning_sig.frame)

				path_planning_sig.processed_start_point
				path_planning_sig.processed_goal_point
				map_params_sig.set_changed (False)
			end
		end


	update_search (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER; path_publisher: separate PATH_PUBLISHER)
			-- Execute search algorithm.
		require
			is_grid_attached
			attached path_planning_sig.start_point
			attached path_planning_sig.goal_point
			not (map_params_sig.is_changed or path_planning_sig.changed_start)
			path_planning_sig.changed_goal
		local
			start_point: POINT
			goal_point: POINT
			start_node: SPATIAL_GRAPH_NODE
			goal_node: SPATIAL_GRAPH_NODE
			open_set: DISPENSER [LABELED_NODE]
			idx: INTEGER
			path: ARRAYED_STACK [SPATIAL_GRAPH_NODE]
		do
			--open_set := create {ARRAYED_QUEUE [LABELED_NODE]}.make (0)

			if attached grid_wrapper as gw and
			   attached path_planning_sig.start_point as sp and
			   attached path_planning_sig.goal_point as gp then

				create start_point.make_from_separate (sp)
				create goal_point.make_from_separate (gp)

--				if path_planning_sig.bfs then
--					open_set := create {LINKED_QUEUE [LABELED_NODE]}.make
--				elseif path_planning_sig.dfs then
--					open_set := create {LINKED_STACK [LABELED_NODE]}.make
--				elseif path_planning_sig.dijkstra then
--					open_set := create {HEAP_PRIORITY_QUEUE [LABELED_NODE]}.make (gw.grid.nodes.count)
--				end

				start_node := gw.grid.node_at (((start_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((start_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)
				goal_node := gw.grid.node_at (((goal_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((goal_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)
				path := search_algorithm.search (start_node, goal_node, path_planning_sig.edge_cost_strategy, path_planning_sig.heuristic_strategy)
				path_publisher.publish_path (path, path_planning_sig.frame)

				path_planning_sig.processed_start_point
				path_planning_sig.processed_goal_point
				map_params_sig.set_changed (False)
			end
		end

feature {NONE} -- Implementation

	grid_wrapper: detachable GRID_FROM_MAP
			-- Grid realization from the map msg.

	search_algorithm: LABEL_CORRECTING_GRAPH_SEARCH
			-- Search algorithm to execute.

end

