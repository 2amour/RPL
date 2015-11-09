note
	description: "Class that publishes odometry msgs to ROS."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	ODOMETRY_PUBLISHER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create `Current' with topic.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, True)
			has_published := False
			timestamp := 0.0
		end

feature {ANY} -- Access

	timestamp: REAL_64
			-- Timestamp of msg

	has_published: BOOLEAN
			-- Is the publisher used

	publish_odometry(a_odometry_sig: separate ODOMETRY_SIGNALER)
			-- Publishing path.
		do
			publisher.publish (a_odometry_sig.data)
			has_published := True
			timestamp := a_odometry_sig.timestamp
		end

	publish_odometry_msg(a_odometry_msg: separate ODOMETRY_MSG)
			-- Publishing path.
		do
			publisher.publish (a_odometry_msg)
			has_published := True
			timestamp := a_odometry_msg.header.timestamp
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [ODOMETRY_MSG]
			-- Publisher object.

end
