note
	description: "Behaviour of the robot implementing tangent bug algorithm."
	author: "Sebastián Curi"
	date: "18.10.15"

class
	TANGENT_BUG_BEHAVIOUR

inherit
	THYMIO_BEHAVIOUR

create
	make_with_attributes

feature {NONE} -- Initialization

	make_with_attributes (parameters_bag: separate TANGENT_BUG_PARAMETERS_BAG)
			-- Create behaviour with given attributes.
		do
			create stop_signaler.make
			create tangent_bug_signaler.make_with_attributes (parameters_bag.goal_parameters, parameters_bag.pid_parameters, parameters_bag.wall_following_parameters)
		end

feature -- Access

	start
			-- Start the behaviour.
		local
			a: separate TANGENT_BUG_CONTROLLER
		do
			create a.make_with_attributes (stop_signaler)
			sep_stop (stop_signaler, False)
			sep_start (a)
		end

	stop
			-- Stop the behaviour.
		do
			sep_stop (stop_signaler, True)
		end

feature {NONE} -- Implementation

	sep_start (a: separate TANGENT_BUG_CONTROLLER)
			-- Start behaviour controllers.
		do
			if attached odometry_sig as a_odometry_sig and
				attached range_group as a_range_group and
				attached ground_group as a_ground_group and
				attached diff_drive as a_diff_drive then
				a.repeat_until_stop_requested (
					agent a.update_velocity (tangent_bug_signaler, a_odometry_sig, a_range_group, a_ground_group, stop_signaler, a_diff_drive))
			end
		end

	sep_stop (stop_sig: separate STOP_SIGNALER; val: BOOLEAN)
			-- Signal behavior for move_toa stop.
		do
			stop_sig.set_stop_requested (val)
		end
end
