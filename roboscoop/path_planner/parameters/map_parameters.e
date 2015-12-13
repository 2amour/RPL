note
	description: "Map processing parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MAP_PARAMETERS

inherit
	PARAMETERS

create
	make_default

feature {NONE} -- Initialization

	make_default
			-- Create `Current' with default values.
		do
			block_width := 1
			block_height := 1
			inflation := 0.0
			connectivity_strategy := create {FULL_CONNECTIVITY_STRATEGY}
		end

feature {ANY} -- Access

	block_width: INTEGER
			-- width of the block in pixels.

	set_block_width(a_block_width: INTEGER)
			-- Set block width.
		do
			block_width := a_block_width
		end

	block_height: INTEGER
			-- height of the block in pixels.

	set_block_height(a_block_height: INTEGER)
			-- Set block height.
		do
			block_height := a_block_height
		end

	inflation: REAL_64
			-- Inflation distance for map boundaries

	set_inflation(a_inflation: REAL_64)
			-- Set map inflation.
		do
			inflation := a_inflation
		end

	connectivity_strategy: GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

	set_connectivity_strategy(a_strategy: GRID_CONNECTIVITY_STRATEGY)
			-- Set connectivity strategy.
		do
			connectivity_strategy := a_strategy
		end

end
