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

hasframes(::Type) = false
hasframes(x::T) where {T} = hasframes(T)

frames_match(::Union{CartesianFrame3D, Nothing}, ::Union{CartesianFrame3D, Nothing}) = true
frames_match(f1::CartesianFrame3D, f2::CartesianFrame3D) = f1 === f2

Base.in(x::CartesianFrame3D, y::CartesianFrame3D) = x == y

"""
$(SIGNATURES)

Check that `CartesianFrame3D` `f1` is one of `f2s`.
Note that if `f2s` is a `CartesianFrame3D`, then `f1` and `f2s` are simply checked for equality.
Throws an `ArgumentError` if `f1` does not match `f2`.
"""
macro framecheck(f1, f2s)
    quote
        $(esc(f1)) ∉ $(esc(f2s)) && framecheck_fail($(QuoteNode(f1)), $(QuoteNode(f2s)), $(esc(f1)), $(esc(f2s)))
    end
end

"""
$(SIGNATURES)

Check that `f1` matches `f2`, where `f1` and `f2` are either `CartesianFrame3D` or `Nothing`.
Throws an `ArgumentError` if `f1` does not match `f2`.
Otherwise, returns whichever of `f1` and `f2` is not nothing, or `nothing` if both are `nothing`.
"""
macro sameframe(f1, f2)
    quote
        if !frames_match($(esc(f1)), $(esc(f2)))
            framecheck_fail($(QuoteNode(f1)), $(QuoteNode(f2)), $(esc(f1)), $(esc(f2)))
        end
        $(esc(f1)) === nothing ? $(esc(f2)) : $(esc(f1))
    end
end

@noinline function framecheck_fail(expr1, expr2, f1::CartesianFrame3D, f2::CartesianFrame3D)
    throw(ArgumentError("$(framecheck_string(expr1, f1)) ≠ $(framecheck_string(expr2, f2))"))
end

framecheck_string(expr, frame::CartesianFrame3D) = "$expr: $(name_and_id(frame))"

const FrameOrNothing = Union{CartesianFrame3D, Nothing}
