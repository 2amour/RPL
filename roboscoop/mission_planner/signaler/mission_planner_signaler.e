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

			io.put_string (way_points.count.out + "%N")
			timestamp := 1
			goal_threshold := thresh
			is_path_requested := True

			create path.make (0)
			path.start

		end

feature {ANY} -- Access

	way_points: ARRAYED_LIST[POINT]
			-- Way points to visit.

	is_path_requested: BOOLEAN
			-- Is a new path requested?

	path: ARRAYED_LIST [separate POINT]
			-- Path of points to visit.

	timestamp: REAL_64
			-- Timestamp of last update.

	goal_threshold: REAL_64
			-- Threshold to switch way_points in path.

	set_timestamp (time: REAL_64)
			-- Set new timestamp
		require
			time > 0
		do
			timestamp := time
		ensure
			timestamp = time
		end

	reset_path
			-- Update path of points.
		do
			create path.make (0)
			path.start
		end

	update_path (point: separate POINT)
			-- Update path of points.
		do
			path.force (create{POINT}.make_from_separate (point))
		end

	request_path (a_val: BOOLEAN)
			-- Set a_val to is_path_requested
		do
			is_path_requested := a_val
		ensure
			is_path_requested = a_val
		end

end
