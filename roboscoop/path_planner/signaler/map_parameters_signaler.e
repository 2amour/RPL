note
	description: "Signaler that contains map-related parameters parameters."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	MAP_PARAMETERS_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_blocking: INTEGER_32; a_inflation: REAL_64; a_connectivity: separate GRID_CONNECTIVITY_STRATEGY)
			-- Make `Current' and assign its attributes.
		do
			blocking := a_blocking
			inflation := a_inflation
			connectivity_strategy := a_connectivity
			timestamp := 0
		end

feature {ANY} -- Access

	blocking: INTEGER_32
			-- Blocking parameter for grid rescaling.


	inflation: REAL_64
			-- Inflation distance for map boundaries.

	connectivity_strategy: separate GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

	timestamp: REAL_64
			-- Timestamp  of last update.

	set_timestamp (a_timestamp: REAL_64)
			-- Set a new timestamp
		do
			timestamp := a_timestamp
		end

end
