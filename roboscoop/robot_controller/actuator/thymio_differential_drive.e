note
	description: "Differential drive of the Thymio. Controls the speed of the robot."
	author: "Rusakov Andrey"
	date: "09.05.2014"

class
	THYMIO_DIFFERENTIAL_DRIVE

inherit
	DIFFERENTIAL_DRIVE

create
	make_with_topic

feature {NONE} -- Initialization

	make_with_topic (topic_name: separate STRING)
			-- Create Current.
		do
			base_size := {THYMIO_ROBOT}.robot_base_size
			max_linear_speed := {THYMIO_ROBOT}.max_linear_speed
			create publisher.make_with_topic (topic_name)
			publisher.advertize (1, False)
		end

feature -- Access

	set_velocity (a_vx: REAL_64; a_vtheta: REAL_64)
			-- Publishing speed.
		local
			linear_speed: REAL_64
			angular_speed: REAL_64
		do
			linear_speed := a_vx
			angular_speed := a_vtheta

			if linear_speed > max_linear_speed then
				linear_speed := max_linear_speed
			end
			if angular_speed > 2.0*max_linear_speed/base_size then
				angular_speed := 2.0*max_linear_speed/base_size
			end
			publisher.publish (create {TWIST_MSG}.make_with_values (
							create {VECTOR_3D_MSG}.make_with_values (linear_speed, 0, 0),
							create {VECTOR_3D_MSG}.make_with_values (0, 0, angular_speed)))
		end

	stop
			-- Publish zero speed.
		do
			set_velocity (0.0, 0.0)
		end

feature {NONE} -- Implementation

	publisher: ROS_PUBLISHER [TWIST_MSG]
			-- Publisher object.
end
