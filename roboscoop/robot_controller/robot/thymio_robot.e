note
	description: "Example class of the Thymio-II robot."
	author: "Rusakov Andrey"
	date: "05.09.2013"

class
	THYMIO_ROBOT

inherit
	BARRIER
	SEPARATE_STRING_MAKER

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS; thymio_robot_behaviour: separate ROBOT_BEHAVIOUR)
			-- Create a robot with range sensors parameters.
		do
			-- Initialize sensors.
			create odometry_signaler.make_with_topic ({THYMIO_TOPICS}.odometry)
			create range_sensors.make ({THYMIO_TOPICS}.prox_horizontal, range_sensors_parameters)
			create ground_sensors.make ({THYMIO_TOPICS}.prox_ground)

			-- Initialize actuators.
			create diff_drive.make_with_topic ({THYMIO_TOPICS}.velocity)

			-- Assign behaviour.
			thymio_robot_behaviour.set_robot_parts (odometry_signaler, range_sensors, ground_sensors, diff_drive)
			behaviour := thymio_robot_behaviour
		end

feature -- Constants

	robot_base_size: REAL_64 = 0.11 -- TODO remove
			-- Robot's width.

	default_linear_speed: REAL_64 = 0.08 -- TODO remove
			-- Default linear speed of the robot.

feature -- Access

	dispatch
			-- Start assigned behaviour.
		do
			start_behaviour (behaviour)
		end

	stop
			-- Stop assigned behaviour.
		do
			stop_behaviour (behaviour)
		end

feature {NONE} -- Robot parts

	range_sensors: separate THYMIO_RANGE_GROUP
			-- Horizontal range sensors.

	ground_sensors: separate THYMIO_GROUND_GROUP
			-- Ground sensors.

	odometry_signaler: separate ODOMETRY_SIGNALER
			-- Current state of the odometry.

	diff_drive: separate THYMIO_DIFFERENTIAL_DRIVE
			-- Differential drive.

feature {NONE} -- Implementation

	behaviour: separate BEHAVIOUR

	start_behaviour (b: separate BEHAVIOUR)
			-- Launch `b'.
		do
			b.start
			io.put_string ("Behaviour started%N")
		end

	stop_behaviour (b: separate BEHAVIOUR)
			-- Terminate `b'.
		do
			b.stop
			synchronize (b)
			io.put_string ("Behaviour requested for a stop%N")
		end
end
