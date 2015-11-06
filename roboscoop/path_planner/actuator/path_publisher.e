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

	publish_path (path: separate LIST [SPATIAL_GRAPH_NODE])
			-- Publishing path.
		do
			publisher.publish (get_path_msg (path))
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [PATH_MSG]
			-- Publisher object.

	get_path_msg (path: separate LIST [SPATIAL_GRAPH_NODE]): separate PATH_MSG
			-- Get path_msg from path feature.
		local
			msg: PATH_MSG
			header: HEADER_MSG
			pose: POSE_MSG
			a_poses: ARRAY [POSE_STAMPED_MSG]
			idx: INTEGER_32
		do
			header := create {HEADER_MSG}.make_now ({PATH_PLANNING_TOPICS}.frame)
			create a_poses.make_filled (create {POSE_STAMPED_MSG}.make_empty, 1, path.count)
			idx := 1
			from
			until
				path.is_empty
			loop
				pose := create {POSE_MSG}.make_with_values (create {POINT_MSG}.make_from_separate (path.item.position), create {QUATERNION_MSG}.make_empty)
				a_poses.put (create {POSE_STAMPED_MSG}.make_with_values (header, pose), idx)
				path.remove
				idx := idx + 1
			end
			create msg.make_with_values (header, a_poses)
			Result := msg
		end

end
