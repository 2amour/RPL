note
	description: "List of used ROS topics for interaction with this path_planner."
	author: "Sebastian Curi"
	date: "28.10.2015"

class
	PATH_PLANNING_TOPICS

feature {ANY} -- Constants

	map: STRING_8 = "/map"
			-- map to be read.

	map_metadata: STRING_8 = "/map_metadata"
			-- metadata of the map.

	frame: STRING_8 = "map"
			-- frame to publish path.

	path: STRING_8 = "/path"
			-- topic where path must be published.

	node: STRING_8 = "/visualization_marker"
			-- marker to show start, goal and current nodes.

end
