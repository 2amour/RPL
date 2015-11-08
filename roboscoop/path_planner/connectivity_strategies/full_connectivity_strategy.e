note
	description: "Strategy for connecting nodes with horizontal, vertical and Diagonal neighbours in a gird."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	FULL_CONNECTIVITY_STRATEGY

inherit

	GRID_CONNECTIVITY_STRATEGY

feature {ANY} -- Access

	connect (g: GRID_GRAPH)
			-- Create connections with neighbours.
		local
			ix, iy, iz: INTEGER_32
		do
			from
				ix := 1
			until
				ix > g.count_x
			loop
				from
					iy := 1
				until
					iy > g.count_y
				loop
					from
						iz := 1
					until
						iz > g.count_z
					loop
						connect_node (g, ix, iy, iz, ix + 1, iy, iz)
					--	connect_node (g, ix, iy, iz, ix - 1, iy, iz)
						connect_node (g, ix, iy, iz, ix, iy + 1, iz)
					--	connect_node (g, ix, iy, iz, ix, iy - 1, iz)
						connect_node (g, ix, iy, iz, ix, iy, iz + 1)
					--	connect_node (g, ix, iy, iz, ix, iy, iz - 1)
						connect_node (g, ix, iy, iz, ix + 1, iy + 1, iz)
					--	connect_node (g, ix, iy, iz, ix - 1, iy + 1, iz)
						connect_node (g, ix, iy, iz, ix + 1, iy - 1, iz)
					--	connect_node (g, ix, iy, iz, ix - 1, iy - 1, iz)
						connect_node (g, ix, iy, iz, ix + 1, iy, iz + 1)
					--	connect_node (g, ix, iy, iz, ix - 1, iy, iz + 1)
						connect_node (g, ix, iy, iz, ix + 1, iy, iz - 1)
					--	connect_node (g, ix, iy, iz, ix - 1, iy, iz - 1)
						connect_node (g, ix, iy, iz, ix, iy + 1, iz + 1)
					--	connect_node (g, ix, iy, iz, ix, iy - 1, iz + 1)
						connect_node (g, ix, iy, iz, ix, iy + 1, iz - 1)
					--	connect_node (g, ix, iy, iz, ix, iy - 1, iz - 1)
						connect_node (g, ix, iy, iz, ix + 1, iy + 1, iz + 1)
						connect_node (g, ix, iy, iz, ix - 1, iy + 1, iz + 1)
						connect_node (g, ix, iy, iz, ix + 1, iy - 1, iz + 1)
						connect_node (g, ix, iy, iz, ix - 1, iy - 1, iz + 1)
					--	connect_node (g, ix, iy, iz, ix + 1, iy + 1, iz - 1)
					--	connect_node (g, ix, iy, iz, ix - 1, iy + 1, iz - 1)
					--	connect_node (g, ix, iy, iz, ix + 1, iy - 1, iz - 1)
					--	connect_node (g, ix, iy, iz, ix - 1, iy - 1, iz - 1)
						iz := iz + 1
					end
					iy := iy + 1
				end
				ix := ix + 1
			end
		end

end
