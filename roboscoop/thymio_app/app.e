note
	description: "Example application of Thymio-II in Roboscoop."
	author: "Rusakov Andrey"
	date: "10.09.2014"

class
	APP

inherit
	ROS_ENVIRONMENT

create
	make

feature {NONE} -- Initialization

	make
			-- Create and launch the robot.
		local
			robo_node: separate ROBOSCOOP_NODE
			ros_spinner: separate ROS_SPINNER
			thymio: separate THYMIO_ROBOT
			target_x: REAL_64
			target_y: REAL_64
		do
			-- Real command line arguments
			if	(arguments.argument (1).is_real_64 and arguments.argument (2).is_real_64) then
				target_x := arguments.argument (1).to_real_64
				target_y := arguments.argument (2).to_real_64
				io.put_string ("x: ")
				io.put_double (target_x)
				io.put_string (", y: ")
				io.put_double (target_y)
				io.put_string ("%N")
			else
				io.put_string ("Error parsing arguments, using x:= 0, y := 0%N")
			end

			-- Initialize this application as a ROS node.
			robo_node := (create {ROS_NODE_STARTER}).roboscoop_node
			synchronize (robo_node)

			-- Listen to ROS.
			create ros_spinner.make
			start_spinning (ros_spinner)

			-- Create a robot object.
			create thymio.make

			-- Launch Thymio.
			separate thymio as t do
				t.move_to (target_x, target_y)
			end
		end
end
