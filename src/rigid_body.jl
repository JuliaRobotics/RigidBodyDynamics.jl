immutable RigidBody{T<:Real}
    name::ASCIIString
    frame::CartesianFrame3D
    inertia::SpatialInertia{T}

    # world body
    RigidBody(name::ASCIIString) = new(name, CartesianFrame3D(name))

    # other bodies
    RigidBody(name::ASCIIString, inertia::SpatialInertia{T}) = new(name, inertia.frame, inertia)
    RigidBody(inertia::SpatialInertia{T}) = new(inertia.frame.name, inertia.frame, inertia) # TODO: deprecate?
end
RigidBody{T}(name::ASCIIString, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia) # TODO: deprecate?
name(b::RigidBody) = b.name
isroot(b::RigidBody) = !isdefined(b, :inertia)
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
