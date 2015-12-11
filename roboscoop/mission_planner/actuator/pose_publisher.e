note
	description: "Class that publishes stamped pose msgs to ROS."
	author: "Sebastian Curi"
	date: "11/12/2015"


class
	POSE_PUBLISHER

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
			-- Was the publisher used

	publish_pose (a_pose: separate POSE)
			-- Publishing a pose.
		do
			publisher.publish (a_pose.get_pose_stamped_msg)
			has_published := True
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [POSE_STAMPED_MSG]
			-- Publisher object.

end
