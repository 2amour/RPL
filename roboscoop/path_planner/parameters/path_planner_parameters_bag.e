note
	description: "Path planner parameters bag that groups different types of parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	PATH_PLANNER_PARAMETERS_BAG

inherit
	PARAMETERS_BAG

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (map_params: MAP_PARAMETERS; path_params: PATH_PLANNER_PARAMETERS; topics: PATH_PLANNER_TOPICS_PARAMETERS)
			-- Create `Current' and assign given attributes.
		do
			map_parameters := map_params
			path_planner_parameters := path_params
			path_planner_topics := topics
		end

feature -- Access

	map_parameters: MAP_PARAMETERS
			-- map parameters.

	path_planner_parameters: PATH_PLANNER_PARAMETERS
			-- Path planner behaviour parameters.

	path_planner_topics: PATH_PLANNER_TOPICS_PARAMETERS
			-- Mission planner communication topics.

end
