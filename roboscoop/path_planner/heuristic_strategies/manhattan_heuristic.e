note
	description: "Manhattan distance heuristic implementation."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	MANHATTAN_HEURISTIC

inherit

	COST_HEURISTIC

feature {ANY} -- Access

	cost (a, b: SPATIAL_GRAPH_NODE): REAL_64
			-- Manhattan distance between two nodes.
		do
			Result := a.manhattan_distance (a.position, b.position)
		end

end
