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

	make_with_attributes (a_frame_id: STRING_8; a_search_strategy: LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY; a_edge_cost, a_heuristic_cost: COST_HEURISTIC)
			-- Create `Current' and assign given attributes.
		do
			frame_id := a_frame_id
			search_strategy := a_search_strategy
			edge_cost := a_edge_cost
			heuristic_cost := a_heuristic_cost
		end

feature {ANY} -- Acces

	frame_id: STRING_8
			-- Frame where path is published.

	edge_cost: COST_HEURISTIC
			-- Edge cost function.

	heuristic_cost: COST_HEURISTIC
			-- Heuristic cost function.

	search_strategy: LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY
			-- Label correcting search strateyg.

end
