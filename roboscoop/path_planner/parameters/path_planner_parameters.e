note
	description: "Path planner parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS

inherit
	PARAMETERS

create
	make_default

feature {NONE} -- Initialization

	make_default
			-- Create `Current' with default values
		do
			frame_id := ""
			search_strategy := create {A_STAR}.make_default
			edge_cost := create {ZERO_HEURISTIC}
			heuristic_cost := create {ZERO_HEURISTIC}
		end

--	make_with_attributes (a_frame_id: STRING; a_search_strategy: LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY; a_edge_cost, a_heuristic_cost: COST_HEURISTIC)
--			-- Create `Current' and assign given attributes.
--		do
--			frame_id := a_frame_id
--			search_strategy := a_search_strategy
--			edge_cost := a_edge_cost
--			heuristic_cost := a_heuristic_cost
--		end

feature {ANY} -- Acces

	frame_id: STRING
			-- Frame where path is published.

	set_frame_id (a_frame: separate STRING)
			-- Set frame id.
		do
			frame_id := create {STRING}.make_from_separate (a_frame)
		end

	edge_cost: COST_HEURISTIC
			-- Edge cost function.

	set_edge_cost (a_edge_cost: COST_HEURISTIC)
			-- Set edge_cost.
		do
			edge_cost := a_edge_cost
		end

	heuristic_cost: COST_HEURISTIC
			-- Heuristic cost function.

	set_heuristic_cost (a_heuristic_cost: COST_HEURISTIC)
			-- Set edge_cost.
		do
			heuristic_cost := a_heuristic_cost
		end

	search_strategy: LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY
			-- Label correcting search strateyg.

	set_search_strategy (a_search_strategy: LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY)
			-- Set search_strategy.
		do
			search_strategy := a_search_strategy
		end

end
