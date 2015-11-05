note
	description: "Wrapper to create a GRID_GRAPH from MAP."
	author: "Sebastian Curi"
	date: "30.10.2015"

class
	GRID_FROM_MAP

create
	make_with_connectivity

feature {NONE} -- Initialization.

	make_with_connectivity (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Make `Current's grid from a map message and a connectivity strategy.
		local
			ix, iy: INTEGER_32
		do
			create grid.make_2d (map.state.info.width.as_integer_32, map.state.info.height.as_integer_32, map.state.info.origin.position.x, map.state.info.origin.position.x + map.state.info.width * map.state.info.resolution, map.state.info.origin.position.y, map.state.info.origin.position.y + map.state.info.height * map.state.info.resolution, map_params_sig.connectivity_strategy)
			map.inflate (map_params_sig.inflation)
			from
				iy := 1
			until
				iy > grid.count_y
			loop
				from
					ix := 1
				until
					ix > grid.count_x
				loop
					if map.state.data.item (ix + (iy - 1) * grid.count_x) > map.occupancy_threshold then
						grid.add_obstacle_by_index (ix, iy, 1)
					end
					ix := ix + 1
				end
				iy := iy + 1
			end
		end

feature {ANY} --Access

	grid: GRID_GRAPH
			-- Grid obtained from inflated map.

end
