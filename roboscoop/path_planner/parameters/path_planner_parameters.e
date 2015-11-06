note
	description: "Path planner parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS

inherit
	PARAMETERS

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_frame_id: STRING_8; a_edge_cost, a_heuristic_cost: COST_HEURISTIC; is_bfs, is_dfs, is_dijkstra: BOOLEAN)
			-- Create `Current' and assign given attributes.
		do
			frame_id := a_frame_id
			edge_cost := a_edge_cost
			heuristic_cost := a_heuristic_cost
			bfs := is_bfs
			dfs := is_dfs
			dijkstra := is_dijkstra
		end

feature {ANY} -- Acces

	frame_id: STRING_8
			-- Frame where path is published.

	edge_cost: COST_HEURISTIC
			-- Edge cost function.

	heuristic_cost: COST_HEURISTIC
			-- Heuristic cost function.

	bfs: BOOLEAN
			-- Breadth-first-search strategy.

	dfs: BOOLEAN
			-- Depth-first-search strategy.

	dijkstra: BOOLEAN
			-- Best-first-search strategy.

end
