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
		end

feature {ANY} -- Access

	publish_path (point_msg: separate POINT_MSG)
			-- Publishing path.
		do
			publisher.publish (point_msg)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [POINT_MSG]
			-- Publisher object.

end
