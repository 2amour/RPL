note
	description: "Files parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	FILES_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty files parameters object.
		do
			create goal_parameters_file_path.make_empty
			create pid_parameters_file_path.make_empty
			create wall_following_parameters_file_path.make_empty
			create range_sensors_parameters_file_path.make_empty
		end

	make_with_attributes (files_goal_parameters_file_path: STRING; files_pid_parameters_file_path: STRING;
							files_wall_following_parameters_file_path: STRING; files_range_sensors_parameters_file_path: STRING)
			-- Create files parameters object with attributes.
		do
			create goal_parameters_file_path.make_from_string (files_goal_parameters_file_path)
			create pid_parameters_file_path.make_from_string (files_pid_parameters_file_path)
			create wall_following_parameters_file_path.make_from_string (files_wall_following_parameters_file_path)
			create range_sensors_parameters_file_path.make_from_string (files_range_sensors_parameters_file_path)
		end

	make_from_separate (other: FILES_PARAMETERS)
			-- Create files parameters object from separate object.
		do
			create goal_parameters_file_path.make_empty
			create pid_parameters_file_path.make_empty
			create wall_following_parameters_file_path.make_empty
			create range_sensors_parameters_file_path.make_empty

			goal_parameters_file_path.copy (other.goal_parameters_file_path)
			pid_parameters_file_path.copy (other.pid_parameters_file_path)
			wall_following_parameters_file_path.copy (other.wall_following_parameters_file_path)
			range_sensors_parameters_file_path.copy (other.range_sensors_parameters_file_path)
		end

feature -- Access

	goal_parameters_file_path: STRING
			-- Goal parameters file path.

	pid_parameters_file_path: STRING
			-- PID parameters file path.

	wall_following_parameters_file_path: STRING
			-- Wall following parameters file path.

	range_sensors_parameters_file_path: STRING
			-- Range sensors parameters file path.

	set_goal_parameters_file_path (files_goal_parameters_file_path: STRING)
			-- Setter for `goal_parameters_file_path'.
		do
			goal_parameters_file_path.copy (files_goal_parameters_file_path)
		end

	set_pid_parameters_file_path (files_pid_parameters_file_path: STRING)
			-- Setter for `pid_parameters_file_path'.
		do
			pid_parameters_file_path.copy (files_pid_parameters_file_path)
		end

	set_wall_following_parameters_file_path (files_wall_following_parameters_file_path: STRING)
			-- Setter for `wall_following_parameters_file_path'.
		do
			wall_following_parameters_file_path.copy (files_wall_following_parameters_file_path)
		end

	set_range_sensors_parameters_file_path (files_range_sensors_parameters_file_path: STRING)
			-- Setter for `range_sensors_parameters_file_path'.
		do
			range_sensors_parameters_file_path.copy (files_range_sensors_parameters_file_path)
		end
end
