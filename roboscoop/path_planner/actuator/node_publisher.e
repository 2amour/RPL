note
	description: "Spatial Graph Node Publisher."
	author: "Sebastian Curi"
	date: "31.10.2015"

class
	NODE_PUBLISHER

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

	publish_node (node: separate SPATIAL_GRAPH_NODE; col: separate COLOR_RGBA_MSG; id: INTEGER_32; scale_value: REAL_64)
			-- Publishing path.
		local
			msg: MARKER_MSG
			header: HEADER_MSG
			pose: POSE_MSG
			scale: VECTOR_3D_MSG
			color: COLOR_RGBA_MSG
		do
			create header.make_now ({PATH_PLANNING_TOPICS}.frame)
			create pose.make_with_values (create {POINT_MSG}.make_from_separate (node.position), create {QUATERNION_MSG}.make_empty)
			create scale.make_with_values (scale_value, scale_value, scale_value)
			create color.make_from_separate (col)
			create msg.make_with_values (header, pose, scale, color, "", "", id, 1, 0, 1000000000)
			publisher.publish (msg)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [MARKER_MSG]
			-- Publisher object.

end
