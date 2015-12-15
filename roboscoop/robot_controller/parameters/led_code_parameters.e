note
	description: "LED CODE."
	author: "Sebastian Curi"
	date: "15.12.2015"

class
	LED_CODE_PARAMETERS

inherit
	PARAMETERS

create
	make, make_with_attributes, make_from_separate

feature {NONE} -- Implementation

	make
			-- Create empty goal parameters object.
		do
			r := 0
			g := 0
			b := 0
		end

	make_with_attributes (a_r, a_g, a_b: DOUBLE)
			-- Create goal parameters object with attributes.
		do
			r := a_r
			g := a_g
			b := a_b
		end

	make_from_separate (other: separate LED_CODE_PARAMETERS)
			-- Create goal parameters object from separate other.
		do
			r := other.r
			g := other.g
			b := other.b
		end

feature -- Access

	r: DOUBLE
			-- Red channel code.

	g: DOUBLE
			-- Green channel code.

	b: DOUBLE
			-- Blue channel code.

	get_code: ARRAY[DOUBLE]
			-- Get RGB code.
		do
			Result := create {ARRAY[DOUBLE]}.make_from_array (<<r, g, b>>)
		end

	set_r (a_r: DOUBLE)
			-- Set red channel.
		do
			r := a_r
		end

	set_g (a_g: DOUBLE)
			-- Set green channel.
		do
			g := a_g
		end

	set_b (a_b: DOUBLE)
			-- Set blue channel.
		do
			b := a_b
		end

	set_code(a_r, a_g, a_b: DOUBLE)
			-- Set color code.
		do
			r := a_r
			g := a_g
			b := a_b
		end

end
