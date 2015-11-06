note
	description: "Summary description for {ROBOT_TARGET_CONTROLLER}."
	author: ""
	date: "$Date$"
	revision: "$Revision$"

class
	ROBOT_TARGET_CONTROLLER

inherit

	CANCELLABLE_CONTROL_LOOP

create
	make

feature {NONE} -- Initialization

	make (s_sig: separate STOP_SIGNALER)
			-- Create `Current' and assign given attributes.
		do
			stop_signaler := s_sig
		end

feature {MISSION_PLANNER_BEHAVIOUR} -- Execute algorithm

	update_target (odometry_sig: separate ODOMETRY_SIGNALER; mission_sig: separate MISSION_PLANNER_SIGNALER; target_pub: separate POINT_PUBLISHER)
			-- update target of robot driver.
		require
			get_separate_point_distance (mission_sig.path.item, (create {POINT}.make_from_msg (odometry_sig.data.pose.pose.position))) < mission_sig.goal_threshold
			get_separate_point_distance (mission_sig.path.item, mission_sig.path.last) > mission_sig.goal_threshold
			-- TODO CHECK IF mission_sig.path.item /= mission_sig.goal works!
		do
			io.put_string ("there?")

			mission_sig.path.remove
			target_pub.publish_point (mission_sig.path.item)
		end

feature {MISSION_PLANNER_BEHAVIOUR} -- Implementation

	get_separate_point_distance (point, other: separate POINT): REAL_64
			-- Get distance from point in separate call
		do
			Result := point.euclidean_distance  (other)
		end



end
