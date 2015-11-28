note
	description: "Resulting path publisher."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	PATH_PUBLISHER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create `Current' with topic.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, True)
		end

feature {ANY} -- Access

	publish_path_from_nodes (path: separate LIST [SPATIAL_GRAPH_NODE]; frame: separate STRING_8)
			-- Publishing path.
		local
			path_getter: PATH_MSG_FROM_NODE_LIST
		do
			create path_getter
			publisher.publish (path_getter.get_path_msg_from_nodes (path, frame))
		end

	publish_path_from_points (path: separate LIST [separate POINT]; frame: separate STRING_8)
			-- Publishing path.
		local
			path_getter: PATH_MSG_FROM_POINT_LIST
		do
			create path_getter
			publisher.publish (path_getter.get_path_msg_from_points (path, frame))
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [PATH_MSG]
			-- Publisher object.
end
