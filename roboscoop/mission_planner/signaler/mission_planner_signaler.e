note
	description: "Signaler with higher-level mission parameters."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	MISSION_PLANNER_SIGNALER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (wp: separate LIST[separate POINT]; thresh: REAL_64)
			-- Make `Current' and assign its attributes.
		do
			way_points := wp

			timestamp := 0
			goal_threshold := thresh
			is_path_requested := True

			create path.make (0)
			path.force (wp.first)
			path.force (wp.last)
			path.start
		end

feature {ANY} -- Access

	way_points: separate LIST[separate POINT]
			-- Way points to visit.

	is_path_requested: BOOLEAN
			-- Is a new path requested?

	path: ARRAYED_LIST [separate POINT]
			-- Path of points to go.

	timestamp: REAL_64
			-- Timestamp of last update.

	goal_threshold: REAL_64
			-- Threshold to switch way_points in path.

	set_timestamp (time: REAL_64)
			-- set new timestamp
		require
			time > 0
		do
			timestamp := time
		ensure
			timestamp = time
		end

	update_path (point: separate POINT)
			-- Update path of points.
		do
			path.extend (create{POINT}.make_from_separate (point))
		end

	request_path (a_val: BOOLEAN)
			-- set a_val to is_path_requested
		do
			is_path_requested := a_val
		ensure
			is_path_requested = a_val
		end
end
