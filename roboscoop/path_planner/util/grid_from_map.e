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
			ix, iy, ixx, iyy: INTEGER_32
			idx: INTEGER_32
			resolution: REAL_64
			width, height: REAL_64
			block_width, block_height: INTEGER_32
			n_pixels_x, n_pixels_y: INTEGER_32
			n_blocks_x, n_blocks_y: INTEGER_32
			block_occupancy: INTEGER_32
		do
			resolution := map.state.info.resolution
			n_pixels_x := map.state.info.width.as_integer_32
			n_pixels_y := map.state.info.height.as_integer_32
			width := resolution * n_pixels_x
			height := resolution * n_pixels_y

			block_width := map_params_sig.block_width
			block_height := map_params_sig.block_height

			n_blocks_x := (n_pixels_x / map_params_sig.block_width).floor
			n_blocks_y := (n_pixels_y / map_params_sig.block_height).floor

			create grid.make_2d (n_blocks_x,
								n_blocks_y,
								map.state.info.origin.position.x + (width/n_blocks_x)/2,
								map.state.info.origin.position.x + width - (width/n_blocks_x)/2,
								map.state.info.origin.position.y + (height/n_blocks_y)/2,
								map.state.info.origin.position.y + height - (height/n_blocks_y)/2,
								map_params_sig.connectivity_strategy)

			from iy := 1 until iy > n_blocks_y
			loop
				from ix := 1 until ix > n_blocks_x
				loop
					block_occupancy := 0
					from iyy := 1 until iyy > block_height
					loop
						from ixx := 1 until ixx > block_width
						loop
							block_occupancy := block_occupancy + map.state.data[((iy - 1)*block_height + iyy - 1)*n_pixels_x + (ix - 1)*block_width + ixx]
							ixx := ixx + 1
						end
						iyy := iyy + 1
					end
					if block_occupancy >= map.occupancy_threshold*(block_height*block_width) then
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
