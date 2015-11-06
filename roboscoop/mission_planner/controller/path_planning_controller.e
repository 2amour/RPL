note
	description: "Summary description for {PATH_PLANNING_CONTROLLER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	PATH_PLANNING_CONTROLLER

inherit

	CANCELLABLE_CONTROL_LOOP

create
	make

feature {NONE} -- Initialization

	make (s_sig: separate STOP_SIGNALER)
			-- Create `Current' and assign given attributes.
		do
			stop_signaler := s_sig
		end

feature {MISSION_PLANNER_BEHAVIOUR} -- Execute algorithm

	update_path (path_sig: separate PATH_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER)
			-- update recieved path.
		require
			path_sig.data.header.timestamp > mission_sig.timestamp
		local
			current_point, next_point: POINT
			idx: INTEGER_32
		do
			io.put_string ("here?")
			mission_sig.update_path (create {POINT}.make_from_msg (path_sig.data.poses[1].pose.position))

			from
				idx := 2
			until
				idx > path_sig.count-1
			loop
				current_point := create {POINT}.make_from_msg (path_sig.data.poses[idx].pose.position)
				next_point := create {POINT}.make_from_msg (path_sig.data.poses[idx+1].pose.position)
				if get_separate_point_angle (mission_sig.path.item, current_point) /= get_separate_point_angle (mission_sig.path.item, next_point) then
					mission_sig.update_path (current_point)
				end
				idx := idx + 1
			end
			mission_sig.update_path (create {POINT}.make_from_msg (path_sig.data.poses[idx+1].pose.position))
			mission_sig.set_timestamp (path_sig.data.header.timestamp)
		end

	request_path (mission_sig: separate MISSION_PLANNER_SIGNALER; start_pub, goal_pub: separate POINT_PUBLISHER)
			-- request a new path to the path_planner.
		require
			mission_sig.is_path_requested
		local
			current_idx: INTEGER_32
		do
			io.put_string ("Publishing!")
			current_idx := get_separate_list_index(mission_sig.way_points)
			start_pub.publish_point (get_separate_list_point (mission_sig.way_points, current_idx))
			goal_pub.publish_point (get_separate_list_point (mission_sig.way_points, current_idx+1))
			mission_sig.request_path (False)
		end

feature {NONE} -- Implementation

	get_separate_point_angle (point, other: separate POINT): REAL_64
			-- Get angle from point in separate call
		do
			Result := point.get_angle (other)
		end

	get_separate_list_index (list: separate LIST[separate POINT]): INTEGER_32
			-- Get index from a separate list
		do
			Result := list.index
		end

	get_separate_list_point (list: separate LIST[separate POINT]; index: INTEGER_32): separate POINT
			-- Get point from a separate list by index
		do
			Result := list.at (index)
		end
end
