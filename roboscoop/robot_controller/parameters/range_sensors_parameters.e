note
	description: "Range sensors parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	RANGE_SENSORS_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty range sensors parameters object.
		do
			sensor_count := 0
			create sensors_poses.make_empty
		end

	make_with_attributes (range_sensors_sensors_poses: ARRAY[POSE_2D])
			-- Create range sensors parameters object with attributes.
		do
			create sensors_poses.make_empty
			sensors_poses.copy (range_sensors_sensors_poses)
			sensor_count := sensors_poses.count
		end

	make_from_separate (other: separate RANGE_SENSORS_PARAMETERS)
			-- Create range sensors parameters object from separate other.
		local
			i: INTEGER
		do
			create sensors_poses.make_filled (create {POSE_2D}.make, other.sensors_poses.lower, other.sensors_poses.upper)
			from i := other.sensors_poses.lower
			until i > other.sensors_poses.upper
			loop
				sensors_poses.put (create {POSE_2D}.make_from_separate (other.sensors_poses[i]), i)
				i := i + 1
			end
			sensor_count := sensors_poses.count
		end

feature -- Access

	sensor_count: INTEGER_32
			-- Number of range sensors.

	sensors_poses: ARRAY[POSE_2D]
			-- Range sensors poses.

	set_sensors_poses (range_sensors_sensors_poses: ARRAY[POSE_2D])
			-- Setter for `sensors_poses'.
		do
			sensors_poses.copy (range_sensors_sensors_poses)
			sensor_count := sensors_poses.count
		end
end
