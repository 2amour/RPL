note
	description: "Roboscoop node with a name, which manages external communication. (Only one node per application can be created.)."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	NAMED_ROBOSCOOP_NODE

create
	init_with_name

feature {NONE} -- Initialization

	init_with_name (a_name: separate STRING_8)
			-- Create a ROS node at the C++ side.
			-- This creation procedure should be called using `ROS_NODE_STARTER' class.
		do
			create node_name.make_from_separate (a_name)
			create c_name.make (Name.twin)
			node := c_ros_new_node (c_name.item)
		end

feature -- Access

	shutdown
			-- Finish and close the node inside ROS.
		do
			c_ros_shutdown_node (node)
		end

feature {NONE} -- Implementation

	node_name: STRING
			-- Name of node in ROS.

	Name: STRING
			-- Name of the node in ROS.
		once
			Result := node_name
		end

	c_name: C_STRING
			-- Node's name object to be used in C++.

	node: POINTER
			-- Pointer to the C++ object.

feature {NONE} -- Externals

	c_ros_new_node (a_c_node_name: POINTER): POINTER
		external
			"C++ inline use %"roboscoop_node.h%""
		alias
			"return new RoboscoopNode($a_c_node_name);"
		end

	c_ros_shutdown_node (a_node_obj: POINTER)
		external
			"C++ inline use %"roboscoop_node.h%""
		alias
			"((RoboscoopNode*)($a_node_obj))->shutdown();"
		end
end
