immutable RigidBody{T<:Real}
    name::AbstractString
    frame::CartesianFrame3D
    isRoot::Bool
    inertia::SpatialInertia{T}

    # world body
    RigidBody(name::AbstractString) = new(name, CartesianFrame3D(name), true)

    # other bodies
    RigidBody(name::AbstractString, inertia::SpatialInertia{T}) = new(name, inertia.frame, false, inertia)
    RigidBody(inertia::SpatialInertia{T}) = new(name(inertia.frame), inertia.frame, false, inertia) # TODO: deprecate?
end
RigidBody{T}(name::AbstractString, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia) # TODO: deprecate?
name(b::RigidBody) = b.name
isroot(b::RigidBody) = b.isRoot
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
