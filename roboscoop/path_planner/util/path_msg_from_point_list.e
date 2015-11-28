note
	description: "Class utility to transform a POINT MSG list into a PATH MSG."
	author: "Sebastian Curi"
	date: "28.11.2015"

class
	PATH_MSG_FROM_POINT_LIST

feature -- Acces

	get_path_msg_from_points (path: separate LIST [separate POINT]; frame: separate STRING_8): separate PATH_MSG
			-- Get path_msg from a list of points.
		local
			msg: PATH_MSG
			header: HEADER_MSG
			point: POINT
			pose: POSE_MSG
			a_poses: ARRAY [POSE_STAMPED_MSG]
			idx: INTEGER_32
		do
			header := create {HEADER_MSG}.make_now (create {STRING_8}.make_from_separate (frame))
			create a_poses.make_filled (create {POSE_STAMPED_MSG}.make_empty, 1, path.count)

			from
				idx := 1
			until
				idx > path.count
			loop
				point := create {POINT}.make_from_separate (path.at (idx))
				pose := create {POSE_MSG}.make_with_values (point.get_msg, create {QUATERNION_MSG}.make_empty)
				a_poses.put (create {POSE_STAMPED_MSG}.make_with_values (header, pose), idx)
				idx := idx + 1
			end
			create msg.make_with_values (header, a_poses)
			Result := msg
		end
end
