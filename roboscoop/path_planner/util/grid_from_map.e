note
	description: "Wrapper to create a GRID_GRAPH from MAP."
	author: "Sebastian Curi"
	date: "30.10.2015"

class
	GRID_FROM_MAP

--			resolution := map.state.info.resolution
--			n_pixels_x := map.state.info.width.as_integer_32
--			n_pixels_y := map.state.info.height.as_integer_32
--			width := resolution * n_pixels_x
--			height := resolution * n_pixels_y

--			n_blocks_x := (n_pixels_x / block_width).floor
--			n_blocks_y := (n_pixels_y / block_height).floor

--			create grid.make_2d (n_blocks_x,
--								n_blocks_y,
--								map.state.info.origin.position.x + (width/n_blocks_x)/2,
--								map.state.info.origin.position.x + width - (width/n_blocks_x)/2,
--								map.state.info.origin.position.y + (height/n_blocks_y)/2,
--								map.state.info.origin.position.y + height - (height/n_blocks_y)/2,
--								connectivity_strategy)

--			from iy := 1 until iy > n_blocks_y
--			loop
--				from ix := 1 until ix > n_blocks_x
--				loop
----					block_occupancy := 0
----					from iyy := 1 until iyy > block_height
----					loop
----						from ixx := 1 until ixx > block_width
----						loop
----							block_occupancy := block_occupancy + map.state.data[((iy - 1)*block_height + iyy - 1)*n_pixels_x + (ix - 1)*block_width + ixx]
----							ixx := ixx + 1
----						end
----						iyy := iyy + 1
----					end
----					if block_occupancy >= map.occupancy_threshold*(block_height*block_width) then
----						grid.add_obstacle_by_index (ix, iy, 1)
----					end
--					if map.state.data.at ((iy - 1) * block_height + ix)  > map.occupancy_threshold then
--						grid.add_obstacle_by_index (ix, iy, 1)
--					end
--					ix := ix + 1
--				end
--				iy := iy + 1
--			end
--		end


feature {ANY} --Access
	get_grid (map: separate OCCUPANCY_GRID_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER): GRID_GRAPH
			-- get a grid.
		local
			ix, iy: INTEGER_32
			a_grid: GRID_GRAPH
		do
			create a_grid.make_2d (map.state.info.width.as_integer_32, map.state.info.height.as_integer_32, map.state.info.origin.position.x, map.state.info.origin.position.x + map.state.info.width * map.state.info.resolution, map.state.info.origin.position.y, map.state.info.origin.position.y + map.state.info.height * map.state.info.resolution, map_params_sig.connectivity_strategy)
			map.inflate (map_params_sig.inflation)
			from
				iy := 1
			until
				iy > a_grid.count_y
			loop
				from
					ix := 1
				until
					ix > a_grid.count_x
				loop
					if map.state.data.item (ix + (iy - 1) * a_grid.count_x) > map.occupancy_threshold then
						a_grid.add_obstacle_by_index (ix, iy, 1)
					end
					ix := ix + 1
				end
				iy := iy + 1
			end
			Result := a_grid
		end
end
