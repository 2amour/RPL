note
	description: "Current state of the point."
	author: "Sebastian Curi"
	date: "07.11.2015"

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
			new_point := False
			create point.make_default

			create subscriber.make
			subscribe_point (subscriber, Current, topic_name)
		end

feature -- Access

	data: POINT_MSG
			-- Current state.

	new_point: BOOLEAN
			-- Is a new point recieved.

	point: POINT
			-- point read

	update_point (msg: separate POINT_MSG)
			-- Update current state with the values from `msg'.
		do
			new_point := not (data.x = msg.x and data.y = msg.y and data.z = msg.z)
			create data.make_from_separate (msg)
			create point.make_from_msg (msg)
		end

	set_new_point (a_val: BOOLEAN)
			-- Update new_point `a_val'.
		do
			new_point := a_val
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
