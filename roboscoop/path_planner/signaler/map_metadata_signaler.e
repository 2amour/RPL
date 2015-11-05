note
	description: "2D grid map metadata."
	author: "Sebastian Curi"
	date: "29.10.2015"

class
	MAP_METADATA_SIGNALER

inherit

	MAP_METADATA_LISTENER

	DOUBLE_MATH
		export
			{NONE} all
		end

create
	make_with_topic

feature {NONE} -- Initilization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			create subscriber.make
			create state.make_empty
			subscribe_map_metadata (subscriber, Current, topic_name)
		end

feature -- Access

	state: MAP_METADATA_MSG
			-- Current data.

	update_map_metadata (metadata: separate MAP_METADATA_MSG)
			-- Update current state with the message's values.
		do
			create state.make_from_separate (metadata)
		end

feature {NONE} -- Implementation

	subscriber: separate ROS_SUBSCRIBER [MAP_METADATA_MSG]
			-- Subscriber object.

	subscribe_map_metadata (a_sub: separate ROS_SUBSCRIBER [MAP_METADATA_MSG]; a_listener: separate MAP_METADATA_LISTENER; a_topic: separate STRING)
			-- Subscriber for map_metadata_update.
		do
			a_sub.subscribe (a_topic, agent a_listener.update_map_metadata)
		end

end
