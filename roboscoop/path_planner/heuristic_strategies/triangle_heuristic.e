note
	description: "Triangle projection heuristic implementation."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	TRIANGLE_HEURISTIC

inherit

	COST_HEURISTIC

feature {ANY} -- Access

	cost (a, b: SPATIAL_GRAPH_NODE): REAL_64
			-- Manhattan distance between two nodes.
		local
			dx, dy: REAL_64
		do
			dx := {DOUBLE_MATH}.dabs (a.position.x - b.position.x)
			dy := {DOUBLE_MATH}.dabs (a.position.y - b.position.y)
			if dx > dy then
				Result := dx + dy * ({DOUBLE_MATH}.sqrt2 - 1)
			else
				Result := dy + dx * ({DOUBLE_MATH}.sqrt2 - 1)
			end
		end

end
