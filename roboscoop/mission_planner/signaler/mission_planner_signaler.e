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

			create path.make (0)
			path.start

			discovered_obstacle := False
		end

feature {ANY} -- Access

	discovered_obstacle: BOOLEAN
			-- Has a new obstacle been discovered?

	way_points: ARRAYED_LIST[POINT]
			-- Way points to visit.

	is_path_requested: BOOLEAN
			-- Is a new path requested?

	path: ARRAYED_LIST [separate POINT]
			-- Path of points to visit.

	goal_threshold: REAL_64
			-- Threshold to switch way_points in path.

	reset_path
			-- Reset path of points.
		do
			create path.make (0)
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


	get_current: POINT
			-- Get current path_point.
		do
			Result := create {POINT}.make_from_separate (path.item)
		end

	get_next: POINT
			-- Get next path_point.
		require
			not path.islast
		do
			path.forth
			Result := get_current
			path.back
		end

	get_origin: POINT
			-- Get robot's origin.
		do
			Result := way_points.first
		end

	get_goal: POINT
			-- Get robot's goal.
		do
			if not way_points.islast then
				way_points.forth
			end
			Result := way_points.item
		end

	set_discovered_obstacle (a_val: BOOLEAN)
			-- Set `a_val' to is_path_requested
		do
			discovered_obstacle := a_val
		end
end
