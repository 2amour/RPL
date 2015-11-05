note
	description: "Mission Planner Topics."
	author: "Sebastian Curi"
	date: "5.11.2015"

class
	MISSION_PLANNER_TOPICS

feature {ANY} -- Constants

	path: STRING_8 = "/path"
			-- topic where path will be read.

	map: STRING_8 = "/map"
			-- map to be published.

	target: STRING_8 = "/driver/goal"
			-- target goal for the driver.

	odometry: STRING_8 = "/thymio_driver/odometry"
			-- odometry topic.

	path_planner_start: STRING_8 = "/path_planner/start"
			-- start position for path_planner node.

	path_planner_goal: STRING_8 = "/path_planner/goal"
			-- goal position for path_planner node.

end
