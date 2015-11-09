note
	description: "Starter for an external Roboscoop named node."
	author: "Sebastian Curi"
	date: "08.11.2015"

class
	ROS_NAMED_NODE_STARTER

inherit
	BARRIER

feature -- Access

	roboscoop_node (name: separate STRING): separate NAMED_ROBOSCOOP_NODE
			-- Singleton for ROS node.
		once ("PROCESS")
			create Result.init_with_name (name)
		end

end
