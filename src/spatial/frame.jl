# NOTE: The `next_frame_id' and `frame_names' globals below are a hack, but they
# enable a significant reduction in allocations.
# Storing the names of all CartesianFrame3D objects in this frame_names vector instead
# of in the CartesianFrame3D and having CartesianFrame3D only contain an integer ID
# makes CartesianFrame3D an isbits (pointer free) type.
# This in turn makes it so that a lot of the geometry/dynamics types become isbits
# types, making them stack allocated and allowing all sorts of
# optimizations.
const next_frame_id = Ref(0)
const frame_names = Dict{Int64, String}()

"""
$(TYPEDEF)

A `CartesianFrame3D` identifies a three-dimensional Cartesian coordinate system.

`CartesianFrame3D`s are typically used to annotate the frame in which certain
quantities are expressed.
"""
struct CartesianFrame3D
    id::Int64

    @doc """
    $(SIGNATURES)

    Create a `CartesianFrame3D` with the given name.
    """ ->
    function CartesianFrame3D(name::String)
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        frame_names[ret.id] = name
        ret
    end

    @doc """
    $(SIGNATURES)

    Create an anonymous `CartesianFrame3D`.
    """ ->
    function CartesianFrame3D()
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        ret
    end
end

Base.print(io::IO, frame::CartesianFrame3D) = print(io, get(frame_names, frame.id, "anonymous"))
name_and_id(frame::CartesianFrame3D) = "\"$frame\" (id = $(frame.id))"
Base.show(io::IO, frame::CartesianFrame3D) = print(io, "CartesianFrame3D: $(name_and_id(frame))")

Base.in(x::CartesianFrame3D, y::CartesianFrame3D) = x == y

"""
$(SIGNATURES)

Check that `CartesianFrame3D` `f1` is one of `f2s`.

Note that if `f2s` is a `CartesianFrame3D`, then `f1` and `f2s` are simply checked for equality.

Throws an `ArgumentError` if `f1` is not among `f2s` when bounds checks
are enabled. `@framecheck` is a no-op when bounds checks are disabled.
"""
macro framecheck(f1, f2s)
    quote
        @boundscheck begin
            $(esc(f1)) ∉ $(esc(f2s)) && framecheck_fail($(QuoteNode(f1)), $(QuoteNode(f2s)), $(esc(f1)), $(esc(f2s)))
        end
    end
end

framecheck_string(expr, frame::CartesianFrame3D) = "$expr: $(name_and_id(frame))"

@noinline function framecheck_fail(expr1, expr2, f1::CartesianFrame3D, f2::CartesianFrame3D)
    throw(ArgumentError("$(framecheck_string(expr1, f1)) ≠ $(framecheck_string(expr2, f2))"))
end

@noinline function framecheck_fail(expr1, expr2, f1::CartesianFrame3D, f2s)
    buf = IOBuffer()
    print(buf, '(')
    first = true
    for f2 in f2s
        first || print(buf, ", ")
        first = false
        print(buf, name_and_id(f2))
    end
    print(buf, ')')
    throw(ArgumentError("$(framecheck_string(expr1, f1)) ∉ $(expr2): $(String(take!(buf)))"))
end
