note
	description: "Current state of a marker."
	author: "Sebastian Curi"
	date: "28.11.2015"

class
	MARKER_SIGNALER

inherit
	MARKER_LISTENER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create data.make_empty
			create subscriber.make
			subscribe_marker (subscriber, Current, topic_name)
			is_new_val := True
		end

feature -- Access

	is_new_val: BOOLEAN
			-- Is a new path recieved.

	data: MARKER_MSG
		-- Current state.

	update_marker (msg: separate MARKER_MSG)
			-- Update current state with the values from `msg'.
		do
			is_new_val := True
			create data.make_from_separate (msg)
		end

	set_new_val (a_val: BOOLEAN)
			-- Set is_new_val to  `a_val'.
		do
			is_new_val := a_val
		end

feature {NONE} -- Implementation

	subscriber: separate ROS_SUBSCRIBER [MARKER_MSG]
			-- Subscriber object.

	subscribe_marker (a_sub: separate ROS_SUBSCRIBER [MARKER_MSG];
							a_listener: separate MARKER_LISTENER; a_topic: separate STRING)
			-- Subscriber for odometry update.
		do
			a_sub.subscribe (a_topic, agent a_listener.update_marker)
		end

end
