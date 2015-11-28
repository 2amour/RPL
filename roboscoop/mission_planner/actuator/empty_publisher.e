note
	description: "Class that publishes point msgs to ROS."
	author: "Sebastian Curi"
	date: "28.11.2015"


class
	EMPTY_PUBLISHER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create `Current' with topic.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, True)
		end

feature -- Acces
	publish
			-- Publishing path.
		do
			publisher.publish (create {EMPTY_MSG}.make_empty)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [EMPTY_MSG]
			-- Publisher object.

end
