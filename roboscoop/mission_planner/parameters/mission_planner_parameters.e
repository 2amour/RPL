note
	description: "Mision planner parameters."
	author: "Sebastian Curi"
	date: "06.11.2015"

class
	MISSION_PLANNER_PARAMETERS

inherit
	PARAMETERS

create
	make_default

feature {NONE} -- Initialization

	make_default
			-- Create `Current' with default values
		do
			create way_points.make (0)
			frame := ""
		end

feature {ANY} -- Acces

	frame: STRING
			-- Global frame

	set_frame (a_frame: STRING)
			-- Set the frame
		do
			frame := a_frame
		end

	way_points: ARRAYED_LIST[POSE]
			-- Way points to visit.

	set_way_points (wp: ARRAYED_LIST[POSE])
			-- Set the way points.
		do
			way_points := wp
		end

	way_point_threshold: REAL_64
			-- Threshold to change way point.

	set_threshold (threshold: REAL_64)
			-- Set threshold to change way point.
		do
			way_point_threshold := threshold
		end

	angle_threshold: REAL_64
			-- Angular hreshold to change way point.

	set_angle_threshold (threshold: REAL_64)
			-- Set anglular threshold to change way point.
		do
			angle_threshold := threshold
		end


	open_loop_offset: REAL_64
			-- Angle offset in radians to put in open loop.

	set_open_loop_offset (offset: REAL_64)
			-- Set angle offset in radians to put in open loop.
		do
			open_loop_offset := offset
		end

end
