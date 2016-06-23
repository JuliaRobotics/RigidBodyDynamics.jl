immutable RigidBody{T<:Real}
    name::String
    frame::CartesianFrame3D
    isRoot::Bool
    inertia::SpatialInertia{T}

    # world body
    RigidBody(name::String) = new(name, CartesianFrame3D(name), true)

    # other bodies
    RigidBody(name::String, inertia::SpatialInertia{T}) = new(name, inertia.frame, false, inertia)
    RigidBody(inertia::SpatialInertia{T}) = new(name(inertia.frame), inertia.frame, false, inertia) # TODO: deprecate?
end
RigidBody{T}(name::String, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia) # TODO: deprecate?
name(b::RigidBody) = b.name
isroot(b::RigidBody) = b.isRoot
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
