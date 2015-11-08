note
	description: "Class that publishes point msgs to ROS."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	POINT_PUBLISHER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create `Current' with topic.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, True)
			has_published := False
		end

feature {ANY} -- Access

	has_published: BOOLEAN
			-- Is the publisher used

	publish_point (a_point: separate POINT)
			-- Publishing path.
		do
			publisher.publish (a_point.get_msg)
			has_published := True
		end

	publish_point2D (a_point: separate POINT_2D)
			-- Publishing path.
		local
			msg: POINT_MSG
		do
			create msg.make_with_values (a_point.get_x, a_point.get_y, 0)
			publisher.publish (msg)
			has_published := True
		end

	publish_point_msg (a_point_msg: separate POINT_MSG)
			-- Publishing path.
		do
			publisher.publish (a_point_msg)
			has_published := True
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [POINT_MSG]
			-- Publisher object.

end
