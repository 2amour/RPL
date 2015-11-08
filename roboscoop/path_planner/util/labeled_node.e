note
	description: "Labeled node for label-correcting algorithm."
	author: "Sebastian Curi"
	date: "30.10.2015"

class
	LABELED_NODE

inherit

	COMPARABLE

create
	make_with_node

feature {NONE} -- Initialization

	make_with_node (n: SPATIAL_GRAPH_NODE)
			-- Make `Current' with node and maximum label.
		do
			node := n
			label := {REAL_64}.max_value
		end

feature {ANY} --Access

	node: SPATIAL_GRAPH_NODE
			-- Graph node.

	label: REAL_64
			-- Current label.

	set_label (new_label: REAL_64)
			-- Set new node label.
		require
			non_negative_label: new_label >= 0
		do
			label := new_label
		ensure
			label_set: label = new_label
		end

	is_less alias "<" (other: like Current): BOOLEAN
			-- Check if other's label is less than current.
		do
			Result := other.label < Current.label
		end

end
