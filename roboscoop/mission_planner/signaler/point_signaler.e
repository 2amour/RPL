note
	description: "Current state of the point."
	author: "Sebastian Curi"
	date: "08.11.2015"


class
	POINT_SIGNALER

inherit
	POINT_LISTENER

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create data.make_empty
			create subscriber.make
			subscribe_point (subscriber, Current, topic_name)
			is_new_val := False
		end

feature -- Access

	is_new_val: BOOLEAN
			-- Is a new path recieved.

	data: POINT_MSG
		-- Current state.

	update_point (msg: separate POINT_MSG)
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

	subscriber: separate ROS_SUBSCRIBER [POINT_MSG]
			-- Subscriber object.

	subscribe_point (a_sub: separate ROS_SUBSCRIBER [POINT_MSG];
							a_listener: separate POINT_LISTENER; a_topic: separate STRING)
			-- Subscriber for odometry update.
		do
			a_sub.subscribe (a_topic, agent a_listener.update_point)
		end
end
