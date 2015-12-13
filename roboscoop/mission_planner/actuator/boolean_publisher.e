note
	description: "Class that publishes bool msgs to ROS."
	author: "Sebastian Curi"
	date: "13.12.2015"

class
	BOOLEAN_PUBLISHER

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
	publish_val(a_val: BOOLEAN)
			-- Publishing a boolean value.
		do
			publisher.publish (create {BOOLEAN_MSG}.make_from_value (a_val))
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [BOOLEAN_MSG]
			-- Publisher object.

end
