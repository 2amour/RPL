note
	description: "Signaler that contains path-planning algorithm parameters."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	PATH_PLANNING_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (w_points: LINKED_LIST[SPATIAL_GRAPH_NODE]; edge_cost: COST_HEURISTIC; heuristic: COST_HEURISTIC; bfs_, dfs_, dijkstra_: BOOLEAN)
			-- Make `Current' and assign its attributes.
		do
			way_points := w_points
			edge_cost_strategy := edge_cost
			heuristic_strategy := heuristic
			bfs := bfs_
			dfs := dfs_
			dijkstra := dijkstra_
		end

feature {ANY} -- Access

	way_points: LINKED_LIST[SPATIAL_GRAPH_NODE]
			-- Way points for path planning algorithm.

	edge_cost_strategy: COST_HEURISTIC
			-- Edge cost function.

	heuristic_strategy: COST_HEURISTIC
			-- Heuristic cost used in a* algorithm.

	bfs: BOOLEAN
			-- Breadth-first-search strategy.

	dfs: BOOLEAN
			-- Depth-first-search strategy.

	dijkstra: BOOLEAN
			-- Best-first-search strategy.

invariant
	only_one: (bfs and not dfs and not dijkstra) or (not bfs and dfs and not dijkstra) or (not bfs and not dfs and dijkstra)

end
