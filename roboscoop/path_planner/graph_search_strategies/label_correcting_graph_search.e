note
	description: "Apply labeling correcting algorithm."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	LABEL_CORRECTING_GRAPH_SEARCH

create
	make_with_graph

feature {NONE} -- Initialization

	make_with_graph (graph: separate GRID_GRAPH)
			-- Make `Current' considering graph size.
		do
				-- Initialize containers
			create cost.make (graph.nodes.count)
			create path.make (graph.nodes.count)
			create path_stack.make (graph.nodes.count)
		end

feature {ANY} -- Access

	search (start_node, goal_node: SPATIAL_GRAPH_NODE; edge_cost: separate COST_HEURISTIC; heuristic: separate COST_HEURISTIC; open: DISPENSER [LABELED_NODE]): ARRAYED_STACK [SPATIAL_GRAPH_NODE]
			-- Execute algorithm.
		local
			current_node: LABELED_NODE
			new_visited_node: LABELED_NODE
			new_cost: REAL_64
		do
			debug
				io.put_string ("start node number of neighbours: " + start_node.neighbours.count.out + "%N")
				io.put_string ("goal node number of neighbours: " + goal_node.neighbours.count.out + "%N")
			end
			new_visited_node := create {LABELED_NODE}.make_with_node (start_node)
			new_visited_node.set_label (0.0)
			open.put (new_visited_node)
			from
			until
				open.is_empty
			loop
				current_node := open.item
				open.remove
				across
					current_node.node.neighbours as neighbour
				loop
					new_cost := cost.at (current_node.node) + edge_cost.cost (current_node.node, neighbour.item)
					if ((not cost.has (goal_node)) or new_cost + heuristic.cost (neighbour.item, goal_node) < cost.at (goal_node)) and ((not cost.has (neighbour.item)) or new_cost < cost.at (neighbour.item)) then
						cost.force (new_cost, neighbour.item)
						path.force (current_node.node, neighbour.item)
						new_visited_node := create {LABELED_NODE}.make_with_node (neighbour.item)
						new_visited_node.set_label (new_cost + heuristic.cost (neighbour.item, goal_node))
						open.put (new_visited_node)
					end
					debug
						if neighbour.item ~ goal_node then
							io.put_string ("Goal Found!%N")
						end
					end
				end
			end
			path_to_array (path, start_node, goal_node)
			io.put_string ("Finished Search!%N")
			Result := path_stack
		end

feature {NONE} -- Implementation

	path: HASH_TABLE [SPATIAL_GRAPH_NODE, SPATIAL_GRAPH_NODE]
			-- Hash table of current_node and next_node for path retrieval.

	cost: HASH_TABLE [REAL_64, SPATIAL_GRAPH_NODE]
			-- Hash table of current_node and next_node for path retrieval.

	path_stack: ARRAYED_STACK [SPATIAL_GRAPH_NODE]
			-- Final retrieved path.

	path_to_array (a_path: HASH_TABLE [SPATIAL_GRAPH_NODE, SPATIAL_GRAPH_NODE]; start_node, goal_node: SPATIAL_GRAPH_NODE)
			-- Retrieve path from (node, parent_node) hash table.
		local
			current_node: SPATIAL_GRAPH_NODE
		do
			if a_path.has (goal_node) and a_path.has (start_node) then
				current_node := goal_node
				from
				until
					current_node ~ start_node
				loop
					path_stack.put (current_node)
					if attached a_path.item (current_node) as new_current_node then
						current_node := new_current_node
					end
				end
			else
				io.put_string ("Goal not found! %N")
				path_stack.put (start_node)
			end
		end

end
