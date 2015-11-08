note
	description: "Map processing parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MAP_PARAMETERS

inherit
	PARAMETERS

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_blocking_width, a_blocking_height: INTEGER_32; a_inflation: REAL_64; a_connectivity: GRID_CONNECTIVITY_STRATEGY)
			-- Create `Current' and assign given attributes.
		do
			block_width := a_blocking_width
			block_height := a_blocking_height
			inflation := a_inflation
			connectivity_strategy := a_connectivity
		end

feature {ANY} -- Acces

	block_width: INTEGER_32
			-- width of the block in pixels.

	block_height: INTEGER_32
			-- height of the block in pixels.

	inflation: REAL_64
			-- Inflation distance for map boundaries

	connectivity_strategy: GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

end
