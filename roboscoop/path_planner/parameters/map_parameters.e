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

	make_with_attributes (a_blocking: INTEGER_32; a_inflation: REAL_64; a_connectivity: GRID_CONNECTIVITY_STRATEGY)
			-- Create `Current' and assign given attributes.
		do
			blocking := a_blocking
			inflation := a_inflation
			connectivity_strategy := a_connectivity
		end

feature {ANY} -- Acces

	blocking: INTEGER_32
			-- Grid blocking for enhanced speed

	inflation: REAL_64
			-- Inflation distance for map boundaries

	connectivity_strategy: GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

end
