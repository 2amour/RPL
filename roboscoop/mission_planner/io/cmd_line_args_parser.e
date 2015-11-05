note
	description: "General class from parsing from the cmd line."
	author: "Sebastian Curi"
	date: "31.10.2015"

deferred class
	CMD_LINE_ARGS_PARSER

feature {ANY} -- Access.

	parse_args (argc: INTEGER_32; argv: ARRAY [STRING_8])
			-- Parse the arguments separately.
		local
			i: INTEGER_32
		do
			from
				i := 1
			until
				i > argc
			loop
				parse (i, argv [i])
				i := i + 1
			end
		end

feature {NONE} -- Implementation.

	parse (idx: INTEGER_32; arg_val: STRING_8)
			-- Parse each argument separately.
		deferred
		end

end
