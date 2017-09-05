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

    """
    $(SIGNATURES)

    Create a `CartesianFrame3D` with the given name.
    """
    function CartesianFrame3D(name::String)
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        frame_names[ret.id] = name
        ret
    end

    """
    $(SIGNATURES)

    Create an anonymous `CartesianFrame3D`.
    """
    function CartesianFrame3D()
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        ret
    end
end

Base.string(frame::CartesianFrame3D) = get(frame_names, frame.id, "anonymous")
Base.show(io::IO, frame::CartesianFrame3D) = print(io, "CartesianFrame3D: \"$(string(frame))\" (id = $(frame.id))")

"""
$(SIGNATURES)

Check that `CartesianFrame3D`s `f1` and `f2` are identical (when bounds checks are enabled).

Throws an `ArgumentError` if `f1` is not identical to `f2` when bounds checks
are enabled. `@framecheck` is a no-op when bounds checks are disabled.
"""
macro framecheck(f1, f2)
    quote
        @boundscheck begin
            $(esc(f1)) != $(esc(f2)) && framecheck_fail($(QuoteNode(f1)), $(QuoteNode(f2)), $(esc(f1)), $(esc(f2)))
        end
    end
end

@noinline function framecheck_fail(sym1, sym2, f1, f2)
    throw(ArgumentError("$(string(sym1)) (\"$(string(f1))\", id = $(f1.id)) ≠ $(string(sym2)) (\"$(string(f2))\", id = $(f2.id))"))
end
