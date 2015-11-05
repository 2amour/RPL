note
	description: "Zero cost heuristic implementation."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	ZERO_HEURISTIC

inherit

	COST_HEURISTIC

feature {ANY} -- Access

	cost (a, b: SPATIAL_GRAPH_NODE): REAL_64
			-- Zero cost.
		do
			Result := 0.0
		end

end
