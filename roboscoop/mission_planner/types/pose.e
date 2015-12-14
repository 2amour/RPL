note
	description: "A 3D pose implementation."
	author: "Sebastian Curi"
	date: "05.11.2015"

class
	POSE

create
	make_default, make_with_values, make_from_separate, make_from_msg, make_from_unstamped_msg

feature {NONE} -- Initialization
	make_default
			-- Make `Current' with default values.
		do
			create position.make_default
			create orientation.make_default
			create frame.make_empty
		end

	make_with_values (a_position: separate POINT; a_orientation: separate QUATERNION; a_frame: separate STRING)
			-- Make `Current' with given values.
		do
			create position.make_from_separate (a_position)
			create orientation.make_from_separate (a_orientation)
			create frame.make_from_separate (a_frame)
		end

	make_from_separate (other: separate like Current)
			-- Make `Current' with given values.
		do
			create position.make_from_separate (other.position)
			create orientation.make_from_separate (other.orientation)
			create frame.make_from_separate (other.frame)
		end

	make_from_msg (msg: separate POSE_STAMPED_MSG)
			-- Make `Current' with given values.
		do
			create position.make_from_msg (msg.pose.position)
			create orientation.make_from_msg (msg.pose.orientation)
			create frame.make_from_separate (msg.header.frame_id)
		end

	make_from_unstamped_msg (msg: separate POSE_MSG; a_frame: separate STRING)
			-- Make `Current' with given values.
		do
			create position.make_from_msg (msg.position)
			create orientation.make_from_msg (msg.orientation)
			create frame.make_from_separate (a_frame)
		end

feature -- Access

	position: POINT
			-- Pose position.

	orientation: QUATERNION
			-- Pose orientation.

	frame: STRING
			-- Pose where frame lives.

	get_pose_msg: POSE_MSG
			-- Get pose msg.
		do
			Result := create {POSE_MSG}.make_with_values (position.get_msg, orientation.get_msg)
		end

	get_pose_stamped_msg: POSE_STAMPED_MSG
			-- Get stamped msg.
		do
			Result := create {POSE_STAMPED_MSG}.make_with_values (create {HEADER_MSG}.make_now (frame), get_pose_msg)
		end

	get_string: STRING
			-- Return string representation of pose.
		do
			Result := "position: " + position.get_string + " orientation: " + orientation.get_string + " frame: "+ frame
		end

	euclidean_distance (other: separate POSE): DOUBLE
			-- Get euclidean distance between two poses.
		do
			Result := position.euclidean_distance (other.position)
		end

	get_angle (other: separate POSE): DOUBLE
			-- Return angle of the vector (in 2D only) connecting two points
		do
			Result := other.orientation.get_theta - orientation.get_theta
		end
end
