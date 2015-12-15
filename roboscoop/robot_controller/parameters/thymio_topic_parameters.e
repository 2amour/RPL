note
	description: "Class with thymio specific topics."
	author: "Sebastian Curi"
	date: "15-12-2015"

class
	THYMIO_TOPIC_PARAMETERS

inherit

	PARAMETERS

create
	make_default, make_from_separate

feature {NONE} -- Initialize

	make_default
			-- Make default
		do
			odometry := {THYMIO_TOPICS}.odometry
			range_sensors := {THYMIO_TOPICS}.prox_horizontal
			ground_sensors := {THYMIO_TOPICS}.prox_ground
			velocity := {THYMIO_TOPICS}.velocity
		end

	make_from_separate (other: separate THYMIO_TOPIC_PARAMETERS)
			-- Make from separate.
		do
			create odometry.make_from_separate (other.odometry)
			create range_sensors.make_from_separate (other.range_sensors)
			create ground_sensors.make_from_separate (other.ground_sensors)
			create velocity.make_from_separate (other.velocity)
		end

feature -- Access

	odometry: STRING
			-- Odometry topic.

	set_odometry (a_topic: separate STRING)
			-- Set odometry topic.
		do
			create odometry.make_from_separate (a_topic)
		end

	range_sensors: STRING
			-- Range sensors topic.

	set_range_sensors (a_topic: separate STRING)
			-- Set range sensors topic.
		do
			create range_sensors.make_from_separate (a_topic)
		end

	ground_sensors: STRING
			-- Ground sensors topic.

	set_ground_sensors (a_topic: separate STRING)
			-- Set ground sensors topic.
		do
			create ground_sensors.make_from_separate (a_topic)
		end

	velocity: STRING
			-- Velocity topic.

	set_velocity (a_topic: separate STRING)
			-- Set velocity topic.
		do
			create velocity.make_from_separate (a_topic)
		end

end
