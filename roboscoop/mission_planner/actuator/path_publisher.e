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
			publisher.advertize (20, False)
		end

feature {ANY} -- Access

	publish_point_path (path: separate LIST [separate POINT]; frame: separate STRING_8)
			-- Publishing path.
		do
			publisher.publish (get_path_msg (path, frame))
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [PATH_MSG]
			-- Publisher object.

	get_path_msg (path: separate LIST [separate POINT]; frame: separate STRING_8): separate PATH_MSG
			-- Get path_msg from points
		local
			msg: PATH_MSG
			header: HEADER_MSG
			pose: POSE_MSG
			a_poses: ARRAY [POSE_STAMPED_MSG]
			idx: INTEGER_32
			point: POINT
		do
			header := create {HEADER_MSG}.make_now (create {STRING_8}.make_from_separate (frame))
			create a_poses.make_filled (create {POSE_STAMPED_MSG}.make_empty, 1, path.count)
			path.start
			from
				idx := 1
			until
				idx > path.count
			loop
				point := create {POINT}.make_from_separate(path.at (idx))
				pose := create {POSE_MSG}.make_with_values (create {POINT_MSG}.make_from_separate (point.get_msg), create {QUATERNION_MSG}.make_empty)
				a_poses.put (create {POSE_STAMPED_MSG}.make_with_values (header, pose), idx)
				idx := idx + 1
			end
			create msg.make_with_values (header, a_poses)
			Result := msg
		end

end
