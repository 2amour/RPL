note
	description: "Signaler that contains path-planning algorithm parameters."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	PATH_PLANNING_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (edge_cost: separate COST_HEURISTIC; heuristic: separate COST_HEURISTIC; bfs_, dfs_, dijkstra_: BOOLEAN; a_frame: separate STRING_8)
			-- Make `Current' and assign its attributes.
		do
			create start_point.make_default
			create goal_point.make_default
			edge_cost_strategy := edge_cost
			heuristic_strategy := heuristic
			bfs := bfs_
			dfs := dfs_
			dijkstra := dijkstra_
			frame := a_frame

			is_search_requested := False
		end

feature {ANY} -- Access

	start_point: POINT
			-- Algorithm start point.

	goal_point: POINT
			-- Algorithm goal point.

	is_search_requested: BOOLEAN
			-- Request a new search

	edge_cost_strategy: separate COST_HEURISTIC
			-- Edge cost function.

	heuristic_strategy: separate COST_HEURISTIC
			-- Heuristic cost used in a* algorithm.

	bfs: BOOLEAN
			-- Breadth-first-search strategy.

	dfs: BOOLEAN
			-- Depth-first-search strategy.

	dijkstra: BOOLEAN
			-- Best-first-search strategy.

	frame: separate STRING_8
			-- Frame id to publish goal.

	request_search (a_val: BOOLEAN)
			-- set `a_val' to is_search_requested.
		do
			is_search_requested := a_val
		end

invariant
	only_one: (bfs and not dfs and not dijkstra) or (not bfs and dfs and not dijkstra) or (not bfs and not dfs and dijkstra)

end
