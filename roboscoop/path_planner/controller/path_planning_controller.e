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
			create search_strategy.make_default
			create grid_wrapper
			create grid.make_1d (1, 0, 0, create {FULL_CONNECTIVITY_STRATEGY})
			stop_signaler := s_sig
		end


feature {PATH_PLANNING_BEHAVIOUR} -- Execute algorithm

	update_start_point (start_sig: separate POINT_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER)
			-- Update start point.
		require
			start_sig.new_point
		do
			io.put_string ("Recieved start point %N")
			path_planning_sig.set_start (start_sig.point)
			start_sig.set_new_point(False)
		end

	update_goal_point (goal_sig: separate POINT_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER)
			-- Update goal point.
		require
			goal_sig.new_point
		do
			io.put_string ("Recieved goal point %N")
			path_planning_sig.set_goal (goal_sig.point)
			goal_sig.set_new_point(False)
		end

	update_map (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Update map.
		require
			map.state.info.resolution > 0
			map.state.header.timestamp > map_params_sig.timestamp
		do
			io.put_string ("Recieved map %N")
			map.inflate (map_params_sig.inflation)
			map_params_sig.set_timestamp(map.state.header.timestamp)
			map_params_sig.set_changed (True)
			map_params_sig.set_created (True)
		end

	search (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER; path_planning_sig: separate PATH_PLANNING_SIGNALER; path_publisher: separate PATH_PUBLISHER)
			-- Execute search algorithm.
		require
			map_params_sig.is_created
			attached path_planning_sig.start_point
			attached path_planning_sig.goal_point
			map_params_sig.is_changed or (path_planning_sig.changed_start and path_planning_sig.changed_goal)
		local
			start_point: POINT
			goal_point: POINT
			start_node: SPATIAL_GRAPH_NODE
			goal_node: SPATIAL_GRAPH_NODE
			open_set: DISPENSER [LABELED_NODE]
			idx: INTEGER
			path: ARRAYED_STACK [SPATIAL_GRAPH_NODE]

		do
			if (attached path_planning_sig.start_point as sp) and
			   (attached path_planning_sig.goal_point as gp) then

				if map_params_sig.is_changed then
					io.put_string ("start building %N")
					grid := grid_wrapper.get_grid (map, map_params_sig)
					map_params_sig.set_changed (False)
					io.put_string ("end building %N")
				end

				create start_point.make_from_separate (sp)
				create goal_point.make_from_separate (gp)

				search_strategy.make_with_size (grid.nodes.count)

				start_node := grid.node_at (((start_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((start_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)
				goal_node := grid.node_at (((goal_point.x - map.state.info.origin.position.x) / map.state.info.resolution).rounded, ((goal_point.y - map.state.info.origin.position.y) / map.state.info.resolution).rounded, 1)

				path := search_strategy.search (start_node, goal_node, path_planning_sig.edge_cost_strategy, path_planning_sig.heuristic_strategy)

				path_publisher.publish_path_from_nodes (path, path_planning_sig.frame)

				path_planning_sig.processed_start_point
				path_planning_sig.processed_goal_point
			end
		end

feature {NONE} -- Implementation

	grid: GRID_GRAPH
			-- Grid graph.

	grid_wrapper: GRID_FROM_MAP
			-- Grid graph wrapper.

	search_strategy: A_STAR -- REMOVE THIS TODO
			-- Search strategy
end

