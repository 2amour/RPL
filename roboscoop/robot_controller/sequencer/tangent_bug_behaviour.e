note
	description: "Behaviour of the robot implementing tangent bug algorithm."
	author: "Sebastian Curi"
	date: "18.10.15"

class
	TANGENT_BUG_BEHAVIOUR

inherit
	ROBOT_BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (ros_topics_params: separate ROBOT_CONTROLLER_TOPIC_PARAMETERS; parameters_bag: separate TANGENT_BUG_PARAMETERS_BAG)
			-- Create behaviour with given attributes.
		do
			create stop_signaler.make
			create tangent_bug_signaler.make_with_attributes (parameters_bag)

			create goal_signaler.make_with_topic (ros_topics_params.goal)

			-- Initialize publihers.
			create odometry_publisher.make_with_topic (ros_topics_params.mission_odometry)
			create obstacles_publisher.make_with_topic (ros_topics_params.sensed_obstacles)

		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate TANGENT_BUG_CONTROLLER
			b, c, d: separate COMMUNICATION_CONTROLLER
		do
			create a.make_with_attributes (stop_signaler)
			create b.make_with_attributes (stop_signaler)
			create c.make_with_attributes (stop_signaler)
			create d.make_with_attributes (stop_signaler)

			sep_stop (stop_signaler, False)
			sep_start (a, b, c, d)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	tangent_bug_signaler: separate TANGENT_BUG_SIGNALER
			-- Signaler for controlling the tangent bug state.

	odometry_publisher: separate ODOMETRY_PUBLISHER
			-- Publisher of current state of the odometry publisher.

	obstacles_publisher: separate POINT_PUBLISHER
			-- Publisher of sensed obstacles.

	goal_signaler: separate POINT_SIGNALER
			-- Signaler to register the goal.

	sep_start (a: separate TANGENT_BUG_CONTROLLER; b, c, d: separate COMMUNICATION_CONTROLLER)
			-- Start behaviour controllers.
		do
			if attached odometry_sig as a_odometry_sig and
				attached range_group as a_range_group and
				attached ground_group as a_ground_group and
				attached diff_drive as a_diff_drive then
				a.repeat_until_stop_requested (
					agent a.update_velocity (tangent_bug_signaler, a_odometry_sig, a_range_group, a_ground_group, stop_signaler, a_diff_drive))
				b.repeat_until_stop_requested (
					agent b.update_goal (goal_signaler, tangent_bug_signaler)) -- TODO - different controllers
				c.repeat_until_stop_requested (
					agent c.publish_odometry (a_odometry_sig, odometry_publisher))
				d.repeat_until_stop_requested (
					agent d.publish_obstacles (a_range_group, obstacles_publisher, a_odometry_sig))
			end
		end

	sep_stop (stop_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Manage stop signaler.
		do
			stop_sig.set_stop_requested (val)
		end
end
