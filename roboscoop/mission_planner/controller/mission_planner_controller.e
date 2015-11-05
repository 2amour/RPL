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
			mission_sig.path.item.euclidean_distance (create {POINT}.make_from_msg (odometry_sig.data.pose.pose.position)) < mission_sig.goal_threshold
		do
			io.put_string ("there?")

			if mission_sig.path.item.euclidean_distance(mission_sig.goal) > mission_sig.goal_threshold then
				mission_sig.path.remove
				target_pub.publish_path (mission_sig.path.item)
			end
		end

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
				if mission_sig.path.item.get_angle (current_point) /= mission_sig.path.item.get_angle (next_point) then
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
		do
			io.put_string ("Publishing!")
			start_pub.publish_path (mission_sig.start)
			goal_pub.publish_path (mission_sig.goal)
			mission_sig.request_path (False)
		end
end
