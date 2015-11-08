note
	description: "Signaler that contains path-planning algorithm parameters."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	PATH_PLANNING_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_search_strategy: separate LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY; edge_cost: separate COST_HEURISTIC; heuristic: separate COST_HEURISTIC; a_frame: separate STRING_8)
			-- Make `Current' and assign its attributes.
		do
			changed_start := False
			changed_goal := False
			search_strategy := a_search_strategy
			edge_cost_strategy := edge_cost
			heuristic_strategy := heuristic
			frame := a_frame
		end

feature {ANY} -- Access

	start_point: detachable POINT
			-- Algorithm start point.

	changed_start: BOOLEAN
			-- Change from last call.

	goal_point: detachable POINT
			-- Algorithm goal point.

	changed_goal: BOOLEAN
			-- Change from last call.

	edge_cost_strategy: separate COST_HEURISTIC
			-- Edge cost function.

	heuristic_strategy: separate COST_HEURISTIC
			-- Heuristic cost used in a* algorithm.

	search_strategy: separate LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY
			-- search strategy.

	frame: separate STRING_8
			-- Frame id to publish goal.

	set_start (point: separate POINT)
			-- set the start.
		do
			if attached start_point as sp then
				changed_start := not (sp.x = point.x and sp.y = point.y and sp.z = point.z)
			else
				changed_start := True
			end
			create start_point.make_from_separate (point)
		end

	processed_start_point
			-- processed change in start point
		do
			changed_start := False
		end

	set_goal (point: separate POINT)
			-- set the goal.
		do
			if attached goal_point as gp then
				changed_goal := not (gp.x = point.x and gp.y = point.y and gp.z = point.z)
			else
				changed_goal := True
			end

			create goal_point.make_from_separate (point)
		end

	processed_goal_point
			-- processed change in goal point
		do
			changed_goal := False
		end

end
