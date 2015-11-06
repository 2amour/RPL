note
	description: "Mission planner parameters bag that groups different types of parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETERS_BAG

inherit

	PARAMETERS_BAG

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (params: MISSION_PLANNER_PARAMETERS;  topics: MISSION_PLANNER_TOPICS_PARAMETERS)
			-- Create `Current' and assign given attributes.
		do
			mission_planner_parameters := params
			mission_planner_topics := topics
		end

feature -- Access

	mission_planner_parameters: MISSION_PLANNER_PARAMETERS
			-- Mission planner behaviour parameters.

	mission_planner_topics: MISSION_PLANNER_TOPICS_PARAMETERS
			-- Mission planner communication topics.

end
