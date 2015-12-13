note
	description: "A quaternion implementation"
	author: "Sebastian Curi"
	date: "11/12/2015"

class
	QUATERNION
create
	make_default, make_with_values, make_from_separate, make_from_msg, make_from_angles, make_from_heading

feature {NONE} -- Initialization

	make_default
			-- Make `Current' with default values.
		do
			x := 0
			y := 0
			z := 0
			w := 1
		end

	make_with_values (a_x, a_y, a_z, a_w: REAL_64)
			-- Make `Current' with given values.
		do
			x := a_x
			y := a_y
			z := a_z
			w := a_w
		end

	make_from_separate (other: separate like Current)
			-- Make `Current' with given values.
		do
			x := other.x
			y := other.y
			z := other.z
			w := other.w
		end

	make_from_msg (msg: separate QUATERNION_MSG)
			-- Make `Current' with given values.
		do
			x := msg.x
			y := msg.y
			z := msg.z
			w := msg.w
		end

	make_from_angles(roll, pitch, yaw: REAL_64)
			-- Make `Current' from roll, pitch, yaw euler angles with order ZYX.
		do
			set_euler (roll, pitch, yaw)
		end

	make_from_heading(theta: REAL_64)
			-- Make `Current' from heading angle.
		do
			x := 0
			y := 0
			z := {DOUBLE_MATH}.sine (theta/2)
			w := {DOUBLE_MATH}.cosine (theta/2)
		end

feature {ANY} -- Access

	x: REAL_64
			-- x vector component.

	y: REAL_64
			-- y vector component.

	z: REAL_64
			-- z vector component.

	w: REAL_64
			-- w scalar component.

	get_msg: QUATERNION_MSG
			-- Get point_msg associated to this point.
		do
			Result := create {QUATERNION_MSG}.make_with_values (x, y, z, w)
		end

	normalize
			-- Normalize quaternion
		local
			norm: REAL_64
		do
			norm := get_norm
			x := x/norm
			y := y/norm
			z := z/norm
			w := w/norm
		end

	set_euler (roll, pitch, yaw: REAL_64)
			-- Set quaternion from roll, pitch, yaw euler angles with order ZYX.
		local
			qx, qy, qz: QUATERNION
		do
			qz := create {QUATERNION}.make_with_values (0, 0, {DOUBLE_MATH}.sine (yaw/2.0), {DOUBLE_MATH}.cosine (yaw/2.0))
			qy := create {QUATERNION}.make_with_values (0, {DOUBLE_MATH}.sine (pitch/2.0), 0, {DOUBLE_MATH}.cosine (pitch/2.0))
			qx := create {QUATERNION}.make_with_values ({DOUBLE_MATH}.sine (roll/2.0), 0, 0, {DOUBLE_MATH}.cosine (roll/2.0))

		 	make_from_separate(qz * qy * qx)
		end

	get_roll: REAL_64
			-- Get roll angle.
		do
			Result := atan2(2*(y*z + x*w), 1 - 2*(x*x + y*y))
		end

	get_pitch: REAL_64
			-- Get pitch angle.
		do
			Result := {DOUBLE_MATH}.arc_sine (2*(w*y - x*z))
		end

	get_yaw: REAL_64
			-- Get yaw angle.
		do
			Result := atan2(2*(x*y + z*w), 1 - 2*(y*y + z*z))
		end

	get_theta: REAL_64
			-- Get heding angle.
		do
			Result := 2.0* atan2(z, w)
		end

	get_norm: REAL_64
			-- Get quaternion norm.
		do
			Result := {DOUBLE_MATH}.sqrt (x*x + y*y + z*z + w*w)
		end

	get_conjugate: QUATERNION
			-- Get conjugate quaterion.
		do
			Result := create {QUATERNION}.make_with_values (-x, -y, -z, w)
		end

	get_reverse: QUATERNION
			-- Get conjugate quaterion.
		do
			Result := create {QUATERNION}.make_with_values (x, y, z, -w)
		end

	get_inverse: QUATERNION
			-- Get inverse quaternion.
		local
			inverse: QUATERNION
		do
			inverse := get_conjugate
			inverse.normalize
			Result := inverse
		end

	sub alias "-" (other: separate QUATERNION): QUATERNION
			-- The result is the addition between the original quatenrion and `other'.
		do
			Result := create {QUATERNION}.make_with_values (x - other.x, y - other.y, z - other.z, w - other.w)
		end

	plus alias "+" (other: separate QUATERNION): QUATERNION
			-- The result is the addition between the original quatenrion and `other'.
		do
			Result := create {QUATERNION}.make_with_values (x + other.x, y + other.y, z + other.z, w + other.w)
		end

	times alias "*" (other: separate QUATERNION): QUATERNION
			-- The result is the product between the original quatenrion and `other'.
		do
			Result := create {QUATERNION}.make_with_values (other.w*x + other.z*y - other.y*z + other.x*w,
														   -other.z*x + other.w*y + other.x*z + other.y*w,
															other.y*x - other.x*y + other.w*z + other.z*w,
														   -other.x*x - other.y*y - other.z*z + other.w*w)
		end

	get_string: STRING
			-- Return string representation of orientation.
		do
			Result := "x: " + x.out + " y: " + y.out + " z: " + z.out + " w: " + w.out
		end

feature {NONE} -- Implementation

	atan2(a_y, a_x: REAL_64): REAL_64
			-- Get roll angle.
		external
			"C++ inline use %"math.h%""
		alias
			"return (double) atan2($a_y, $a_x);"
		end
end
