note
	description: "Euclidean distance heuristic implementation."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	EUCLIDEAN_HEURISTIC

inherit

	COST_HEURISTIC

feature {ANY} -- Access

	cost (a, b: SPATIAL_GRAPH_NODE): REAL_64
			-- Euclidean distance between two nodes.
		do
			Result := a.euclidean_distance (a.position, b.position)
		end

end
