@indextype BodyID

"""
$(TYPEDEF)

A non-deformable body.

A `RigidBody` has inertia (represented as a [`SpatialInertia`](@ref)),
unless it represents a root (world) body. A `RigidBody` additionally stores
a list of definitions of coordinate systems that are rigidly attached to it.
"""
mutable struct RigidBody{T}
    name::String
    inertia::Union{SpatialInertia{T}, Nothing}
    frame_definitions::Vector{Transform3D{T}}
    contact_points::Vector{DefaultContactPoint{T}} # TODO: allow different contact models
    id::BodyID

    # inertia undefined; can be used for the root of a kinematic tree
    function RigidBody{T}(name::String) where {T}
        frame = CartesianFrame3D(name)
        new{T}(name, nothing, [one(Transform3D{T}, frame)], DefaultContactPoint{T}[], BodyID(-1))
    end

    # other bodies
    function RigidBody(name::String, inertia::SpatialInertia{T}) where {T}
        new{T}(name, inertia, [one(Transform3D{T}, inertia.frame)], DefaultContactPoint{T}[], BodyID(-1))
    end
end

Base.eltype(::Type{RigidBody{T}}) where {T} = T
Base.eltype(body::RigidBody) = eltype(typeof(body))
RigidBody(inertia::SpatialInertia) = RigidBody(string(inertia.frame), inertia)
Base.print(io::IO, b::RigidBody) = print(io, b.name)

function Base.show(io::IO, b::RigidBody)
    if get(io, :compact, false)
        print(io, b)
    else
        print(io, "RigidBody: \"$(string(b))\"")
    end
end

BodyID(b::RigidBody) = b.id
Base.convert(::Type{BodyID}, b::RigidBody) = BodyID(b)
@inline RigidBodyDynamics.Graphs.vertex_id_type(::Type{<:RigidBody}) = BodyID
@inline RigidBodyDynamics.Graphs.vertex_id(b::RigidBody) = convert(BodyID, b)
@inline RigidBodyDynamics.Graphs.set_vertex_id!(b::RigidBody, id::BodyID) = (b.id = id)

"""
$(SIGNATURES)

Whether the body has a defined inertia.
"""
has_defined_inertia(b::RigidBody) = b.inertia !== nothing

"""
$(SIGNATURES)

Return the spatial inertia of the body. If the inertia is undefined, calling
this method will result in an error.
"""
spatial_inertia(b::RigidBody{T}) where {T} = b.inertia::SpatialInertia{T}

"""
$(SIGNATURES)

Set the spatial inertia of the body.
"""
function spatial_inertia!(body::RigidBody, inertia::SpatialInertia)
    body.inertia = transform(inertia, frame_definition(body, inertia.frame))
end

"""
    frame_definitions(body)

Return the list of homogeneous transforms ([`Transform3D`](@ref)s) that define the
coordinate systems attached to `body` with respect to a single common frame
([`default_frame(body)`](@ref)).
"""
frame_definitions(body::RigidBody) = body.frame_definitions

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
default_frame(body::RigidBody) = body.frame_definitions[1].to # allows standardization on a frame to reduce number of transformations required

"""
$(SIGNATURES)

Return the [`Transform3D`](@ref) defining `frame` (attached to `body`) with
respect to [`default_frame(body)`](@ref).

Throws an error if `frame` is not attached to `body`.
"""
function frame_definition(body::RigidBody, frame::CartesianFrame3D)
    for transform in body.frame_definitions
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
function add_frame!(body::RigidBody, transform::Transform3D)
    # note: overwrites any existing frame definition
    # transform.to needs to be among (transform.from for transform in frame_definitions(body))
    definitions = body.frame_definitions
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
function change_default_frame!(body::RigidBody, new_default_frame::CartesianFrame3D)
    if new_default_frame != default_frame(body)
        old_to_new = inv(frame_definition(body, new_default_frame))
        map!(tf -> old_to_new * tf, body.frame_definitions, body.frame_definitions)
        if has_defined_inertia(body)
            body.inertia = transform(spatial_inertia(body), old_to_new)
        end
        for point in contact_points(body)
            point.location = old_to_new * point.location
        end
    end
end

"""
$(SIGNATURES)

Add a new contact point to the rigid body
"""
function add_contact_point!(body::RigidBody{T}, point::DefaultContactPoint{T}) where {T}
    loc = location(point)
    tf = fixed_transform(body, loc.frame, default_frame(body))
    point.location = tf * loc
    push!(body.contact_points, point)
    nothing
end

"""
$(SIGNATURES)

Return the contact points attached to the body as an ordered collection.
"""
contact_points(body::RigidBody) = body.contact_points
