note
	description: "Current state of the empty msg."
	author: "Sebastian Curi"
	date: "12.12.2015"

class
	EMPTY_SIGNALER

inherit
	EMPTY_LISTENER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create subscriber.make
			subscribe_empty (subscriber, Current, topic_name)
			is_new_val := False
		end

feature -- Access.

	is_new_val: BOOLEAN
			-- Is a new path recieved.

	set_new_val (a_val: BOOLEAN)
			-- Set is_new_val to  `a_val'.
		do
			is_new_val := a_val
		end

	update_state (msg: separate EMPTY_MSG)
			-- Update current state with the values from `msg'.
		do
			is_new_val := True
		end

feature {NONE} -- Implementation

	subscriber: separate ROS_SUBSCRIBER [EMPTY_MSG]
			-- Subscriber object.

	subscribe_empty (a_sub: separate ROS_SUBSCRIBER [EMPTY_MSG];
							a_listener: separate EMPTY_LISTENER; a_topic: separate STRING)
			-- Subscriber for odometry update.
		do
			a_sub.subscribe (a_topic, agent a_listener.update_state)
		end

end
