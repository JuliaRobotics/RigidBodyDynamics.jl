"""
$(TYPEDEF)

A non-deformable body.

A `RigidBody` has inertia (represented as a [`SpatialInertia`](@ref)),
unless it represents a root (world) body. A `RigidBody` additionally stores
a list of definitions of coordinate systems that are rigidly attached to it.
"""
type RigidBody{T<:Number}
    name::String
    inertia::Nullable{SpatialInertia{T}}
    frameDefinitions::Set{Transform3D{T}}
    id::Int64

    # inertia undefined; can be used for the root of a kinematic tree
    function (::Type{RigidBody{T}}){T<:Number}(name::String)
        frame = CartesianFrame3D(name)
        new{T}(name, Nullable{SpatialInertia{T}}(), Set([Transform3D{T}(frame)]), -1)
    end

    # other bodies
    function (::Type{RigidBody{T}}){T<:Number}(name::String, inertia::SpatialInertia{T})
        new{T}(name, Nullable(inertia), Set([Transform3D{T}(inertia.frame)]), -1)
    end
end

Base.eltype{T}(::Type{RigidBody{T}}) = T
Base.eltype{T}(body::RigidBody{T}) = eltype(typeof(body))
RigidBody{T}(name::String, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(name(inertia.frame), inertia)
name(b::RigidBody) = b.name
Base.show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
Base.showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
RigidBodyDynamics.Graphs.vertex_index(b::RigidBody) = b.id
RigidBodyDynamics.Graphs.vertex_index!(b::RigidBody, id::Int64) = (b.id = id)

"""
$(SIGNATURES)

Whether the body has a defined inertia.
"""
has_defined_inertia(b::RigidBody) = !isnull(b.inertia)

"""
$(SIGNATURES)

Return the spatial inertia of the body. If the inertia is undefined, calling
this method will result in an error.
"""
spatial_inertia(b::RigidBody) = get(b.inertia)

"""
$(SIGNATURES)

Set the spatial inertia of the body.
"""
function spatial_inertia!(body::RigidBody, inertia::SpatialInertia)
    body.inertia = Nullable(transform(inertia, frame_definition(body, inertia.frame)))
end

"""
    frame_definitions(body)

Return the list of homogeneous transforms ([`Transform3D`](@ref)s) that define the
coordinate systems attached to `body` with respect to a single common frame
([`default_frame(body)`](@ref)).
"""
frame_definitions(body::RigidBody) = body.frameDefinitions

"""
$(SIGNATURES)

Whether `frame` is attached to `body` (i.e. whether it is among
[`frame_definitions(body)`](@ref)).
"""
is_fixed_to_body(body::RigidBody, frame::CartesianFrame3D) = any((t) -> t.from == frame, frame_definitions(body))

"""
$(SIGNATURES)

The [`CartesianFrame3D`](@ref) with respect to which all other frames attached
to `body` are defined.

See [`frame_definitions(body)`](@ref), [`frame_definition(body, frame)`](@ref).
"""
default_frame(body::RigidBody) = first(frame_definitions(body)).to # allows standardization on a frame to reduce number of transformations required

"""
$(SIGNATURES)

Return the [`Transform3D`](@ref) defining `frame` (attached to `body`) with
respect to [`default_frame(body)`](@ref).

Throws an error if `frame` is not attached to `body`.
"""
function frame_definition(body::RigidBody, frame::CartesianFrame3D)
    for transform in body.frameDefinitions
        transform.from == frame && return transform
    end
    error("$frame not found among body fixed frame definitions for $body")
end

"""
$(SIGNATURES)

Return the transform from `CartesianFrame3D` `from` to `to`, both of which are
rigidly attached to `body`.
"""
function fixed_transform(body::RigidBody, from::CartesianFrame3D, to::CartesianFrame3D)
    transform = frame_definition(body, from)
    if transform.to != to
        transform = inv(frame_definition(body, to)) * transform
    end
    transform
end

"""
$(SIGNATURES)

Add a new frame definition to `body`, represented by a homogeneous transform
from the `CartesianFrame3D` to be added to any other frame that is already
attached to `body`.
"""
function add_frame!{T}(body::RigidBody{T}, transform::Transform3D{T})
    # note: overwrites any existing frame definition
    # transform.to needs to be among (transform.from for transform in frame_definitions(body))
    definitions = body.frameDefinitions
    if transform.to != default_frame(body)
        transform = frame_definition(body, transform.to) * transform
    end
    filter!(t -> t.from != transform.from, definitions)
    push!(definitions, transform)
    transform
end

"""
$(SIGNATURES)

Change the default frame of `body` to `frame` (which should already be among
`body`'s frame definitions).
"""
function change_default_frame!(body::RigidBody, newDefaultFrame::CartesianFrame3D)
    if newDefaultFrame != default_frame(body)
        oldToNew = inv(frame_definition(body, newDefaultFrame))

        # change body-fixed frame definitions so that they transform to the new default frame
        newDefinitions = similar(body.frameDefinitions)
        for tf in body.frameDefinitions
            push!(newDefinitions, oldToNew * tf)
        end
        body.frameDefinitions = newDefinitions
        # map!(tf -> oldToNew * tf, body.frameDefinitions) # TODO: not defined; open issue

        # transform body's spatial inertia to new default frame
        if has_defined_inertia(body)
            body.inertia = Nullable(transform(spatial_inertia(body), oldToNew))
        end
    end
end
