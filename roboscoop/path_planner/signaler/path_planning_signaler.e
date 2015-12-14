note
	description: "Signaler that contains path-planning algorithm parameters."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	PATH_PLANNING_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_search_strategy: separate LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY; edge_cost: separate COST_HEURISTIC; heuristic: separate COST_HEURISTIC)
			-- Make `Current' and assign its attributes.
		do
			changed_start := False
			changed_goal := False
			search_strategy := a_search_strategy
			edge_cost_strategy := edge_cost
			heuristic_strategy := heuristic
		end

feature {ANY} -- Access

	start_pose: detachable POSE
			-- Algorithm start pose.

	changed_start: BOOLEAN
			-- Change from last call.

	goal_pose: detachable POSE
			-- Algorithm goal pose.

	changed_goal: BOOLEAN
			-- Change from last call.

	edge_cost_strategy: separate COST_HEURISTIC
			-- Edge cost function.

	heuristic_strategy: separate COST_HEURISTIC
			-- Heuristic cost used in a* algorithm.

	search_strategy: separate LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY
			-- search strategy.

	set_start (point: separate POSE)
			-- set the start.
		do
			if attached start_pose as sp then
				changed_start := not (sp.position.x = point.position.x and sp.position.y = point.position.y and sp.position.z = point.position.z)
			else
				changed_start := True
			end
			create start_pose.make_from_separate (point)
		end

	processed_start_point
			-- processed change in start point
		do
			changed_start := False
		end

	set_goal (point: separate POSE)
			-- set the goal.
		do
			if attached goal_pose as gp then
				changed_goal := not (gp.position.x = point.position.x and gp.position.y = point.position.y and gp.position.z = point.position.z)
			else
				changed_goal := True
			end

			create goal_pose.make_from_separate (point)
		end

	processed_goal_point
			-- processed change in goal point
		do
			changed_goal := False
		end

end
