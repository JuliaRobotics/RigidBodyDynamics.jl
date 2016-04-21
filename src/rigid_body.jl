immutable RigidBody{T}
    frame::CartesianFrame3D
    inertia::SpatialInertia{T}

    # world body
    RigidBody(name::ASCIIString) = new(CartesianFrame3D(name))

    # other bodies
    RigidBody(inertia::SpatialInertia{T}) = new(inertia.frame, inertia)
end
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia)
name(b::RigidBody) = b.frame.name
isroot(b::RigidBody) = !isdefined(b, :inertia)
