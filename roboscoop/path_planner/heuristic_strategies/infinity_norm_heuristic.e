note
	description: "Infinity norm distance between two points."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	INFINITY_NORM_HEURISTIC

inherit

	COST_HEURISTIC

feature {ANY} -- Access

	cost (a, b: SPATIAL_GRAPH_NODE): REAL_64
			-- Infinity norm distance between two nodes.
		local
			dx, dy: REAL_64
		do
			dx := {DOUBLE_MATH}.dabs (a.position.x - b.position.x)
			dy := {DOUBLE_MATH}.dabs (a.position.y - b.position.y)
			if dx > dy then
				Result := dx
			else
				Result := dy
			end
		end

end
