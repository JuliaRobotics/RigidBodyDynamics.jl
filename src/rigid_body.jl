type RigidBody{T<:Real}
    name::String
    frame::CartesianFrame3D
    inertia::Nullable{SpatialInertia{T}}

    # inertia undefined; can be used for the root of a kinematic tree
    RigidBody(name::String) = new(name, CartesianFrame3D(name), Nullable{SpatialInertia{T}}())

    # other bodies
    RigidBody(inertia::SpatialInertia{T}) = new(name(inertia.frame), inertia.frame, Nullable(inertia))
    RigidBody(name::String, inertia::SpatialInertia{T}) = new(name, inertia.frame, Nullable(inertia))
end

RigidBody{T}(name::String, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(inertia)
Base.@pure eltype{T}(::Type{RigidBody{T}}) = T
@inline eltype{T}(::RigidBody{T}) = eltype(typeof(T))
name(b::RigidBody) = b.name
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
has_defined_inertia(b::RigidBody) = !isnull(b.inertia)
spatial_inertia(b::RigidBody) = get(b.inertia)
