note
	description: "Execute asynchronously the path planning algorithm."
	author: "Sebastian Curi"
	date: "29.10.2015"

class
	PATH_PLANNING_BEHAVIOUR

inherit

	BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (path_planning_sig: separate PATH_PLANNING_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Create Current with signaler.
		do
			create map_signaler.make_with_topic ({PATH_PLANNING_TOPICS}.map)
			create path_publisher.make_with_topic ({PATH_PLANNING_TOPICS}.path)
			create node_publisher.make_with_topic ({PATH_PLANNING_TOPICS}.node)
			create stop_signaler.make
			path_planning_signaler := path_planning_sig
			map_parameters_signaler := map_params_sig
		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate PATH_PLANNING_CONTROLLER
		do
			publish_way_points (map_signaler, path_planning_signaler, node_publisher)
			create a.make (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a, path_planning_signaler, map_parameters_signaler)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	path_planning_signaler: separate PATH_PLANNING_SIGNALER
			-- Signaler with path planning algorithm states.

	map_parameters_signaler: separate MAP_PARAMETERS_SIGNALER
			-- Signaler with input map parameters.

	map_signaler: separate OCCUPANCY_GRID_SIGNALER
			-- Signaler with map data.

	path_publisher: separate PATH_PUBLISHER
			-- Publisher of resultig path.

	node_publisher: separate NODE_PUBLISHER
			-- Publisher of resultig path.

	stop_signaler: separate STOP_SIGNALER
			-- Signaler for stopping the behaviour.

	sep_start (a: separate PATH_PLANNING_CONTROLLER; path_plan_sig: separate PATH_PLANNING_SIGNALER; map_params_sig: separate MAP_PARAMETERS_SIGNALER)
			-- Start controllers asynchronously.
		do
			a.search (map_signaler, map_params_sig, path_planning_signaler, path_publisher)
		end

	sep_stop (s_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for a stop.
		do
			s_sig.set_stop_requested (val)
		end

	publish_way_points (map_sig: separate OCCUPANCY_GRID_SIGNALER; path_plann_sig: separate PATH_PLANNING_SIGNALER; node_pub: separate NODE_PUBLISHER)
			-- Publish start and goal markers.
		require
			map_sig.state.info.resolution > 0
		local
			idx: INTEGER_32
			colour: COLOR_RGBA_MSG
		do
			from
				idx := 1
			until
				idx > path_plann_sig.way_points.count
			loop
				if idx = 1 then -- Start point
					colour := create {COLOR_RGBA_MSG}.make_red
				elseif idx = path_plann_sig.way_points.count then -- Goal node
					colour := create {COLOR_RGBA_MSG}.make_green
				else -- Middle node
					colour := create {COLOR_RGBA_MSG}.make_yellow
				end
				node_pub.publish_node (path_plann_sig.way_points.at (idx), colour, idx, 2 * map_sig.state.info.resolution)
				idx := idx + 1
			end
		end

end
