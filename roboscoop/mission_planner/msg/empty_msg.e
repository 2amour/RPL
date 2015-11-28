note
	description: "Empty ROS msg."
	author: "Sebastian Curi"
	date: "28.11.2015"

class
	EMPTY_MSG

inherit
	ROS_MESSAGE

create
	make_empty, make_from_pointer, make_from_separate

feature {NONE} -- Initialization.

	make_empty
			-- Create empty Current.
		do
		end

	make_from_pointer (c_ptr: POINTER)
			-- Create `Current' by parsing C++ class.
		do
			set_from_pointer (c_ptr)
		end

feature {ROS_SUBSCRIBER}

	set_from_pointer (c_ptr: POINTER)
			-- Create `Current' by parsing C++ class.
		do
		end

feature {ROS_MESSAGE}

	allocate_c_msg: POINTER
			-- Pointer to the corresponding ROS message instance in C++.
		do
			Result := c_msg_ptr ()
		end

	deallocate_c_msg (a_msg_ptr: POINTER)
		external
			"C++ inline use %"std_msgs/Empty.h%""
		alias
			"delete (std_msgs::Empty*)$a_msg_ptr;"
		end

feature {NONE} -- Externals

	get_data_from_c (c_ptr: POINTER)
		external
			"C++ inline use %"std_msgs/Empty.h%""
		alias
			"return;"
		end

	c_msg_ptr (): POINTER
		external
			"C++ inline use %"std_msgs/Empty.h%""
		alias
			"[
				std_msgs::Empty* msg = new std_msgs::Empty();
				return msg;
			]"
		end

	c_ros_advertize (a_worker_obj: POINTER; a_queue_size: INTEGER; a_is_latched: BOOLEAN)
		external
			"C++ inline  use %"publisher.h%", %"std_msgs/Empty.h%""
		alias
			"((Publisher*)($a_worker_obj))->advertize <std_msgs::Empty> ($a_queue_size, $a_is_latched);"
		end

	c_ros_publish (a_worker_obj: POINTER; a_msg_ptr: POINTER)
		external
			"C++ inline use %"publisher.h%", %"std_msgs/Empty.h%""
		alias
			"((Publisher*)($a_worker_obj))->publish_message <std_msgs::Empty> (*((std_msgs::Empty*)$a_msg_ptr));"
		end

	subscribe_to_ros (a_worker_obj: POINTER; c_topic_name: POINTER; obj: ANY; routine: POINTER)
		external
			"C++ inline use %"subscriber.h%", %"std_msgs/Empty.h%""
		alias
			"((Subscriber*)($a_worker_obj))->subscribe<std_msgs::Empty, std_msgs::Empty::ConstPtr> ($c_topic_name, $obj, $routine);"
		end

end
