note
	description: "Breadth-first-search implementation of label correcting algorithm."
	author: "Sebastian Curi"
	date: "07.11.2015"

class
	BFS

inherit
	LABEL_CORRECTING_GRAPH_SEARCH_STRATEGY

create
	make_with_size, make_default

feature {NONE} -- Initialization

	make_default
			-- Make default.
		do
			create cost.make (0)
			create path.make (0)
			create path_stack.make (0)
			open := create {ARRAYED_QUEUE [LABELED_NODE]}.make (0)
		end

feature {ANY} -- Reset

	make_with_size (a_size: INTEGER_32)
			-- Make `Current' considering graph size.
		do
				-- Initialize containers
			size := a_size
			create cost.make (a_size)
			create path.make (a_size)
			create path_stack.make (a_size)
			open := create {ARRAYED_QUEUE [LABELED_NODE]}.make (a_size)
		end
end
