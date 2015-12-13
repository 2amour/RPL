note
	description: "Class that publishes map msgs to ROS."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	OCCUPANCY_GRID_PUBLISHER

create
	make_with_topic

feature {NONE} -- Initialization.

	make_with_topic (topic_name: separate STRING)
			-- Create `Current' with topic.
		do
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, True)
			has_published := False
			frame := ""
		end

feature {ANY} -- Access.

	frame: STRING
			-- Frame of map msg.

	has_published: BOOLEAN
			-- Is the publisher used.

	publish_map (sep_msg: separate OCCUPANCY_GRID_MSG)
			-- Publishing path.
		local
			a_msg, msg: OCCUPANCY_GRID_MSG
		do
			create a_msg.make_from_separate (sep_msg)
			msg := create {OCCUPANCY_GRID_MSG}.make_from_values (
				create {HEADER_MSG}.make_now (frame), a_msg.info, a_msg.data)
			publisher.publish (msg)
			has_published := True
		end

	set_frame(a_frame: separate STRING)
			-- Set the frame's name.
		do
			frame := create {STRING}.make_from_separate (a_frame)
		end

feature {NONE} -- Implementation.

	publisher: ROS_PUBLISHER [OCCUPANCY_GRID_MSG]
			-- Publisher object.

end
