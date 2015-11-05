note
	description: "path_planning application root class"
	date: "28.10.2015"

class
	APP

inherit

	ROS_ENVIRONMENT

	ARGUMENTS

create
	make

feature {NONE} -- Initialization

	make
			-- Run application.
		local
			parser: PATH_PLANNING_PARSER
			path_planner_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			path_planning_signaler: PATH_PLANNING_SIGNALER
			map_parameters_signaler: MAP_PARAMETERS_SIGNALER
			path_planning_behaviour: PATH_PLANNING_BEHAVIOUR
			way_points: LINKED_LIST [SPATIAL_GRAPH_NODE]
		do
				-- Parse command line arguments
			create parser.make
			parser.parse_args (argument_count, argument_array)

				-- Transform LIST[POINT_MSGS] to LIST[SPATIAL_GRAPH_NODES]
			create way_points.make
			across
				parser.way_points_parser.points as points
			loop
				way_points.put_right (create {SPATIAL_GRAPH_NODE}.make_with_coords (points.item))
			end

				-- Initialize this application as a ROS node.
			path_planner_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (path_planner_node)

				-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

				-- Start Signalers
			create path_planning_signaler.make_with_attributes (way_points, parser.edge_cost_parser.cost, parser.heuristic_cost_parser.cost, parser.open_set_strategy_parser.bfs, parser.open_set_strategy_parser.dfs, parser.open_set_strategy_parser.dijkstra)
			create map_parameters_signaler.make_with_attributes (parser.map_inflation_parser.inflation, parser.connectivity_parser.connectivity)

				-- Start Behaviour
			create path_planning_behaviour.make_with_attributes (path_planning_signaler, map_parameters_signaler)
			path_planning_behaviour.start
		end

end
