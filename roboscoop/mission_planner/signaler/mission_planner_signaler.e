note
	description: "Signaler with higher-level mission parameters."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (a_frame: separate STRING; wp: separate ARRAYED_LIST[separate POSE]; thresh: REAL_64)
			-- Make `Current' and assign its attributes.
		local
			i: INTEGER_32
		do
			create frame.make_from_separate (a_frame)
			create way_points.make (wp.count)
			from i := 1
			until i > wp.count
			loop
				way_points.force (create {POSE}.make_from_separate (wp.at (i)))
				i := i + 1
			end
			way_points.start

			goal_threshold := thresh
			is_path_requested := True
			is_obj_recognition_requested := False

			create path.make (0)
			create way_points_idx.make (0)
			path.start

			discovered_obstacle := False
		end

feature {ANY} -- Access

	frame: STRING
			-- Global mission frame.

	discovered_obstacle: BOOLEAN
			-- Has a new obstacle been discovered?

	way_points: ARRAYED_LIST[POSE]
			-- Way points to visit.

	way_points_idx: ARRAYED_LIST[INTEGER]
			-- Array that match idx of path to way point.

	is_path_requested: BOOLEAN
			-- Is a new path requested?

	is_obj_recognition_requested: BOOLEAN
			-- Is a new object recognition requested?

	path: ARRAYED_LIST [separate POSE]
			-- Path of points to visit.

	goal_threshold: REAL_64
			-- Threshold to switch way_points in path.

	is_waypoint_reached: BOOLEAN
			-- Whether a waypoint has benn reached.

	set_waypoint_reached (value: BOOLEAN)
			-- Setter for `is_waypoint_reached'.
		do
			is_waypoint_reached := value
		end

	update_path (pose: separate POSE)
			-- Add `pose' to path.
		do
			path.force (create{POSE}.make_from_separate (pose))
		end

	request_path (a_val: BOOLEAN)
			-- Set `a_val' to is_path_requested
		do
			is_path_requested := a_val
		ensure
			is_path_requested = a_val
		end

	request_object_recognition (a_val: BOOLEAN)
			-- Set `a_val' to is_obj_recognition_requested
		do
			is_obj_recognition_requested := a_val
		ensure
			is_obj_recognition_requested = a_val
		end

	get_current_path_pose: POSE
			-- Get current path_pose.
		do
			Result := create {POSE}.make_from_separate (path.item)
		end

	set_way_point_idx
			-- Set way point index in vector
		do
			way_points_idx.force (path.count)
			way_points_idx.start
		end

	set_discovered_obstacle (a_val: BOOLEAN)
			-- Set `a_val' to is_path_requested
		do
			discovered_obstacle := a_val
		end

	at_ith_way_point (test_point: separate POINT; way_point_idx: INTEGER): BOOLEAN
			-- Check if the point is at one of the way points
		do
			Result := way_points.at (way_point_idx).position.euclidean_distance (test_point) < goal_threshold
		end

	at_a_way_point (test_point: separate POINT): BOOLEAN
			-- Check if the point is at one of the way points
		local
			i: INTEGER
		do
			Result := False
			from i := 2
			until i > way_points.count
			loop
				if not Result then
					Result := at_ith_way_point(test_point, i)
				end
				i := i + 1
			end
		end
end
