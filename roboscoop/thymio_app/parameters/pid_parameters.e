note
	description: "PID parameters."
	author: "Ferran Pallarès"
	date: "20.10.2015"

class
	PID_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty PID parameters object.
		do
		end

	make_with_attributes (pid_kp: REAL_64; pid_ki: REAL_64; pid_kd: REAL_64)
			-- Create PID parameters object with attributes.
		do
			kp := pid_kp
			ki := pid_ki
			kd := pid_kd
		end

	make_from_separate (other: separate PID_PARAMETERS)
			-- Create PID parameters object from separate other.
		do
			kp := other.kp
			ki := other.ki
			kd := other.kd
		end

feature -- Access

	kp: REAL_64
			-- PID proportional gain.

	ki: REAL_64
			-- PID integral gain.

	kd: REAL_64
			-- PID derivative gain.

	set_kp (pid_kp: REAL_64)
			-- Setter for `kp'.
		do
			kp := pid_kp
		end

	set_ki (pid_ki: REAL_64)
			-- Setter for `ki'.
		do
			ki := pid_ki
		end

	set_kd (pid_kd: REAL_64)
			-- Setter for `kd'.
		do
			kd := pid_kd
		end
end
