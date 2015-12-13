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

	update_map (obstacle_sig: separate POINT_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER; map_sig: separate OCCUPANCY_GRID_SIGNALER; map_pub: separate OCCUPANCY_GRID_PUBLISHER; s_sig: separate STOP_SIGNALER)
			-- update map from sensor measurements.
		require
			(obstacle_sig.is_new_val or not map_pub.has_published)
			map_sig.state.info.resolution > 0
		local
			idx: INTEGER
		do
			if not s_sig.is_stop_requested then
				if obstacle_sig.is_new_val then
					idx := ((obstacle_sig.data.y - map_sig.state.info.origin.position.y) / map_sig.state.info.resolution).ceiling
					idx := 1 + (idx-1) * map_sig.state.info.width.as_integer_32 + ((obstacle_sig.data.x - map_sig.state.info.origin.position.x) / map_sig.state.info.resolution).rounded

					if map_sig.state.data.at (idx) < map_sig.occupancy_threshold then
						mission_sig.set_discovered_obstacle (True)
						map_sig.state.data.force ((2*map_sig.occupancy_threshold).as_integer_8, idx)
					end
				end

				if not map_pub.has_published then
					map_pub.publish_map (map_sig.state)
				end

				obstacle_sig.set_new_val (False)
			end
		end

	update_target (odometry_sig: separate ODOMETRY_SIGNALER; localization_sig: separate BOOLEAN_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER; target_pub: separate POSE_PUBLISHER; object_rec_sig: separate EMPTY_SIGNALER; s_sig: separate STOP_SIGNALER)
			-- update target of robot driver.
		require
			not mission_sig.is_path_requested
			not mission_sig.path.islast
			mission_sig.path.count > 0
			object_rec_sig.is_new_val
			not mission_sig.is_obj_recognition_requested
		local
			current_point: POINT
		do
			if not s_sig.is_stop_requested then
				create current_point.make_from_msg (odometry_sig.data.pose.pose.position)
				mission_sig.set_localized_handled (False)

				if localization_sig.data then
					if mission_sig.at_a_way_point (current_point) then
						mission_sig.request_localization (False)
						if mission_sig.is_waypoint_reached then
							mission_sig.request_object_recognition (True)
							mission_sig.set_waypoint_reached (False)
							if not mission_sig.way_points_idx.islast then
								mission_sig.way_points_idx.forth
							end
						end
					else
						mission_sig.set_waypoint_reached (True)
					end

					if mission_sig.discovered_obstacle then
						mission_sig.path.go_i_th (mission_sig.way_points_idx.item)
						target_pub.publish_pose (mission_sig.get_current_path_pose)

						mission_sig.set_discovered_obstacle (False)
					else
						if current_point.euclidean_distance(mission_sig.get_current_path_pose.position) < mission_sig.goal_threshold then
							mission_sig.path.forth
							target_pub.publish_pose (mission_sig.get_current_path_pose)
						end
					end

					if not target_pub.has_published then
						target_pub.publish_pose (create {POSE}.make_default)
					end
				else
					mission_sig.request_localization (True)
					target_pub.publish_pose (create {POSE}.make_with_values (current_point,
						create {QUATERNION}.make_from_heading (odometry_sig.theta + {DOUBLE_MATH}.pi_2), mission_sig.frame))
				end
			end
		end

	update_path (mission_sig: separate MISSION_PLANNER_SIGNALER; path_sig: separate PATH_SIGNALER_WITH_FLAG; s_sig: separate STOP_SIGNALER)
			-- update recieved path.
		require
			path_sig.is_new_val
			path_sig.data.poses.count > 1
			not mission_sig.is_path_requested

		local
			path: ARRAYED_STACK[POINT]
			angles: ARRAYED_STACK[REAL_64]
			current_angle: REAL_64
			current_point, next_point, following_point: POINT
			idx: INTEGER_32
		do
			if not s_sig.is_stop_requested then
				path_sig.set_new_val (False)
				create path.make (0)
				create angles.make (0)
				io.put_string ("Recieved path size: " + path_sig.data.poses.count.out + "%N")

				current_point := create {POINT}.make_from_msg (path_sig.data.poses[1].pose.position)
				current_angle := path_sig.data.poses[1].pose.orientation.theta

				path.put (current_point)
				angles.put (current_angle)
				from
					idx := 2
				until
					idx > path_sig.count-1
				loop
					next_point := create {POINT}.make_from_msg (path_sig.data.poses[idx].pose.position)
					following_point := create {POINT}.make_from_msg (path_sig.data.poses[idx+1].pose.position)

					if {DOUBLE_MATH}.dabs (current_point.get_angle (next_point) - current_point.get_angle (following_point)) < {TRIGONOMETRY_MATH}.pi_16 and
					   current_point.euclidean_distance (next_point) > mission_sig.goal_threshold
					then
						path.put (current_point)
						angles.put (current_point.get_angle (next_point))
						current_point := create {POINT}.make_from_separate (next_point)
					end
					idx := idx + 1
				end
				path.put (create {POINT}.make_from_msg (path_sig.data.poses[idx].pose.position))
				angles.put (path_sig.data.poses[idx].pose.orientation.theta)


				from
				until (path.is_empty or angles.empty)
				loop
					mission_sig.update_path (create{POSE}.make_with_values (path.item, create {QUATERNION}.make_from_heading (angles.item), mission_sig.frame))
					path.remove
					angles.remove
				end
				mission_sig.set_way_point_idx
				io.put_string ("Processed path size: " + mission_sig.path.count.out + "%N")

				if not mission_sig.way_points.islast then
					mission_sig.request_path (True)
				else
					mission_sig.request_path (False)
				end
			end
		end

	request_path (mission_sig: separate MISSION_PLANNER_SIGNALER; obstacle_sig: separate POINT_SIGNALER; start_pub, goal_pub: separate POINT_PUBLISHER; s_sig: separate STOP_SIGNALER)
			-- request a new path to the path_planner.
		require
			not mission_sig.path.islast
			mission_sig.is_path_requested
			not obstacle_sig.is_new_val

		local
			current_idx: INTEGER_32
		do
			if not s_sig.is_stop_requested then
				io.put_string ("Request path%N")
				current_idx := mission_sig.way_points.index
				-- This are inverted! also reinvert in reconstruction
				goal_pub.publish_point (mission_sig.way_points.at (current_idx).position)
				start_pub.publish_point (mission_sig.way_points.at (current_idx+1).position)
				mission_sig.way_points.forth
				mission_sig.request_path (False)
			end

		end

	request_recognition (object_rec_pub: separate EMPTY_PUBLISHER; object_rec_sig: separate EMPTY_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER; odometry_sig: separate ODOMETRY_SIGNALER; s_sig: separate STOP_SIGNALER)
			-- Request the obstacle recognition
		require
			mission_sig.is_obj_recognition_requested
			object_rec_sig.is_new_val
			not (odometry_sig.is_moving or odometry_sig.is_twisting)
		do
			if not s_sig.is_stop_requested then
				io.put_string ("Recognition requested%N")
				mission_sig.request_object_recognition (False)
				object_rec_sig.set_new_val (False)
				object_rec_pub.publish
			end
		end

	request_localization (localization_pub: separate BOOLEAN_PUBLISHER; mission_sig: separate MISSION_PLANNER_SIGNALER; s_sig: separate STOP_SIGNALER)
			-- Start or stop the localization the localization
		require
			not mission_sig.is_loc_request_handled
		do
			if not s_sig.is_stop_requested then
				localization_pub.publish_val (mission_sig.is_localization_requested)
				mission_sig.set_localized_handled (True)
			end
		end

end
