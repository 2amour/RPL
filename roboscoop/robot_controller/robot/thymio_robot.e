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

	make_with_attributes (range_sensors_parameters: separate RANGE_SENSORS_PARAMETERS; thymio_robot_behaviour: separate THYMIO_BEHAVIOUR)
			-- Create a robot with range sensors parameters.
		do
			-- Initialize sensors.
			create range_sensors.make ({THYMIO_TOPICS}.prox_horizontal, range_sensors_parameters)
			create ground_sensors.make ({THYMIO_TOPICS}.prox_ground)
			create odometry_signaler.make_with_topic ({THYMIO_TOPICS}.odometry)

			-- Initialize actuators.
			create diff_drive.make_with_topic ({THYMIO_TOPICS}.velocity)
			create sound_player.make_with_topic ({THYMIO_TOPICS}.sound)
			create top_leds.make_with_topic ({THYMIO_TOPICS}.top_leds)
			create buttons_leds.make_with_topic ({THYMIO_TOPICS}.buttons_leds)
			create circle_leds.make_with_topic ({THYMIO_TOPICS}.circle_leds)

			-- Assign behaviour.
			thymio_robot_behaviour.set_robot_parts (odometry_signaler, range_sensors, ground_sensors, diff_drive, buttons_leds, circle_leds, top_leds)
			behaviour := thymio_robot_behaviour
		end

feature -- Constants

	robot_base_size: REAL_64 = 0.11
			-- Robot's width.

	default_linear_speed: REAL_64 = 0.08
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

	sound_player: separate THYMIO_SOUND_PLAYER
			-- Built-in sound player.

	top_leds: separate THYMIO_TOP_LEDS
			-- RGB LEDs on the top.

	buttons_leds: separate THYMIO_BUTTONS_LEDS
			-- 4 Red LEDs on the buttons.

	circle_leds: separate THYMIO_CIRCLE_LEDS
			-- 8 Orange LEDS around the buttons.

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
