note
	description: "Summary description for {MISSION_PLANNER_CONTROLLER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	MISSION_PLANNER_CONTROLLER

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

	update_target (odometry_sig: separate ODOMETRY_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER; target_pub: separate POINT_PUBLISHER)
			-- update target of robot driver.
		require
			not mission_sig.is_path_requested
			not mission_sig.path.islast
			mission_sig.path.count > 0
		do
			if (create {POINT}.make_from_msg (odometry_sig.data.pose.pose.position)).euclidean_distance(mission_sig.path.item) < mission_sig.goal_threshold then
				mission_sig.path.forth
				target_pub.publish_point (mission_sig.path.item)
			end

			if not target_pub.has_published then
				target_pub.publish_point (mission_sig.path.first)
			end
		end

	update_path (mission_sig: separate MISSION_PLANNER_SIGNALER; path_sig: separate PATH_SIGNALER_WITH_FLAG)
			-- update recieved path.
		require
			path_sig.is_new_val
			path_sig.data.poses.count > 1
			not mission_sig.is_path_requested
		local
			current_point, next_point, following_point: POINT
			idx: INTEGER_32
		do
			path_sig.set_new_val (False)
			io.put_string (mission_sig.timestamp.out + "%N")
			io.put_string (path_sig.data.header.timestamp.out + "%N")

			io.put_string ("Recieved path size: ")
			io.put_string (path_sig.data.poses.count.out + "%N")
			mission_sig.update_path (create {POINT}.make_from_msg (path_sig.data.poses[1].pose.position))
			current_point := create {POINT}.make_from_separate (mission_sig.path.item)

			from
				idx := 2
			until
				idx > path_sig.count-1
			loop
				next_point := create {POINT}.make_from_msg (path_sig.data.poses[idx].pose.position)
				following_point := create {POINT}.make_from_msg (path_sig.data.poses[idx+1].pose.position)
				if {DOUBLE_MATH}.dabs (current_point.get_angle (next_point) - current_point.get_angle (following_point)) < {TRIGONOMETRY_MATH}.pi_16 and
				   current_point.euclidean_distance (next_point) > 4*mission_sig.goal_threshold
				then
					mission_sig.update_path (next_point)
					current_point := create {POINT}.make_from_separate (next_point)
				end
				idx := idx + 1
			end
			mission_sig.update_path (create {POINT}.make_from_msg (path_sig.data.poses[idx].pose.position))

			io.put_string ("Processed path size: " + mission_sig.path.count.out + "%N")
			mission_sig.set_timestamp (path_sig.data.header.timestamp)

			if not mission_sig.way_points.islast then
				mission_sig.request_path (True)
			else
				mission_sig.request_path (False)
			end
		end

	request_path (mission_sig: separate MISSION_PLANNER_SIGNALER; start_pub, goal_pub: separate POINT_PUBLISHER)
			-- request a new path to the path_planner.
		require
			mission_sig.is_path_requested
		local
			current_idx: INTEGER_32
		do
			io.put_string ("Request a path!")
			current_idx := mission_sig.way_points.index
			start_pub.publish_point (mission_sig.way_points.at (current_idx))
			goal_pub.publish_point (mission_sig.way_points.at (current_idx+1))
			mission_sig.way_points.forth
			mission_sig.request_path (False)
		end

end
