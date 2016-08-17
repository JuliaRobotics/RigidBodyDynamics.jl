immutable RigidBody{T<:Real}
    name::String
    frame::CartesianFrame3D
    inertia::SpatialInertia{T}

    # inertia undefined; can e.g. be used for the root of a kinematic tree
    RigidBody(name::String) = new(name, CartesianFrame3D(name))

    # other bodies
    RigidBody(inertia::SpatialInertia{T}) = new(name(inertia.frame), inertia.frame, inertia)
    RigidBody(name::String, inertia::SpatialInertia{T}) = new(name, inertia.frame, inertia)
end

RigidBody{T}(name::String, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia)
name(b::RigidBody) = b.name
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
has_defined_inertia(b::RigidBody) = isdefined(b, :inertia)
