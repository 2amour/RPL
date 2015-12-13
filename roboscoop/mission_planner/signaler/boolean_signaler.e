note
	description: "Current state of the boolean msg."
	author: "Sebastian Curi"
	date: "12.12.2015"

class
	BOOLEAN_SIGNALER

inherit
	BOOLEAN_LISTENER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create subscriber.make
			subscribe_empty (subscriber, Current, topic_name)
			data := False
		end

feature -- Access

	data: BOOLEAN

	update_state (msg: separate BOOLEAN_MSG)
			-- Update current state with the values from `msg'.
		do
			data := msg.data
		end

feature {NONE} -- Implementation

	subscriber: separate ROS_SUBSCRIBER [BOOLEAN_MSG]
			-- Subscriber object.

	subscribe_empty (a_sub: separate ROS_SUBSCRIBER [BOOLEAN_MSG];
							a_listener: separate BOOLEAN_LISTENER; a_topic: separate STRING)
			-- Subscriber for odometry update.
		do
			a_sub.subscribe (a_topic, agent a_listener.update_state)
		end

end
