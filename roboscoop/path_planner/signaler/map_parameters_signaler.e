note
	description: "Signaler that contains map-related parameters parameters."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	MAP_PARAMETERS_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_blocking_width, a_blocking_height: INTEGER_32; a_inflation: REAL_64; a_connectivity: separate GRID_CONNECTIVITY_STRATEGY)
			-- Make `Current' and assign its attributes.
		do
			block_width := a_blocking_width
			block_height := a_blocking_height
			inflation := a_inflation
			connectivity_strategy := a_connectivity
			timestamp := 0
			is_changed := False
			is_created := False
		end

feature {ANY} -- Access

	block_width: INTEGER_32
			-- width of the block in pixels.

	block_height: INTEGER_32
			-- height of the block in pixels.

	inflation: REAL_64
			-- Inflation distance for map boundaries.

	connectivity_strategy: separate GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

	is_created: BOOLEAN
			-- Has the map been initialized.

	is_changed: BOOLEAN
			-- Has the map changed?

	timestamp: REAL_64
			-- Timestamp  of last update.

	set_timestamp (a_timestamp: REAL_64)
			-- Set timestap to `a_timestamp'.
		do
			timestamp := a_timestamp
		end

	set_changed (a_val: BOOLEAN)
			-- Set is_changed to `a_val'.
		do
			is_changed := a_val
		end

	set_created (a_val: BOOLEAN)
			-- Set is_created to `a_val'.
		do
			is_created := a_val
		end

end
