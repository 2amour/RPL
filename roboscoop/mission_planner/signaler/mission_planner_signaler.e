note
	description: "Signaler with higher-level mission parameters."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (wp: separate ARRAYED_LIST[separate POINT]; thresh: REAL_64)
			-- Make `Current' and assign its attributes.
		local
			print_point: POINT
			i: INTEGER_32
		do
			create way_points.make (wp.count)
			from i := 1
			until i > wp.count
			loop
				way_points.force (create {POINT}.make_from_separate (wp.at (i)))
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

	discovered_obstacle: BOOLEAN
			-- Has a new obstacle been discovered?

	way_points: ARRAYED_LIST[POINT]
			-- Way points to visit.

	way_points_idx: ARRAYED_LIST[INTEGER]
			-- Array that match idx of path to way point.

	is_path_requested: BOOLEAN
			-- Is a new path requested?

	is_obj_recognition_requested: BOOLEAN
			-- Is a new object recognition requested?

	path: ARRAYED_LIST [separate POINT]
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

	reset_path
			-- Reset path of points.
		do
			create path.make (0)
			create way_points_idx.make(0)
			path.start
		end

	update_path (point: separate POINT)
			-- Add `point' to path.
		do
			path.force (create{POINT}.make_from_separate (point))
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


	get_current_path_point: POINT
			-- Get current path_point.
		do
			Result := create {POINT}.make_from_separate (path.item)
		end

	get_next_path_point: POINT
			-- Get next path_point.
		require
			not path.islast
		do
			path.forth
			Result := get_current_path_point
			path.back
		end

	get_next_way_point: POINT
			-- Get next path_point.
		do
			Result := create {POINT}.make_from_separate (path.at(way_points_idx.item))
		end

	get_origin: POINT
			-- Get robot's origin.
		do
			Result := way_points.first
		end

	get_goal: POINT
			-- Get robot's goal.
		do
			Result := way_points.item
		end

	set_way_point_idx
			-- Set way point index in vector
		do
			way_points_idx.force (path.count)
		end

	set_discovered_obstacle (a_val: BOOLEAN)
			-- Set `a_val' to is_path_requested
		do
			discovered_obstacle := a_val
		end

	at_ith_way_point (test_point: separate POINT; way_point_idx: INTEGER): BOOLEAN
			-- Check if the point is at one of the way points
		do
			Result := way_points.at (way_point_idx).euclidean_distance (test_point) < goal_threshold
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
