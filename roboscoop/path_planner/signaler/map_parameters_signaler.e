note
	description: "Signaler that contains map-related parameters parameters."
	author: "Sebastian Curi"
	date: "01.11.2015"

class
	MAP_PARAMETERS_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (inf: REAL_64; connectivity: GRID_CONNECTIVITY_STRATEGY)
			-- Make `Current' and assign its attributes.
		do
			inflation := inf
			connectivity_strategy := connectivity
		end

feature {ANY} -- Access

	inflation: REAL_64
			-- Inflation distance for map boundaries

	connectivity_strategy: GRID_CONNECTIVITY_STRATEGY
			-- Grid connectivity used.

end
