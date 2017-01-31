type RigidBody{T<:Number}
    name::String
    inertia::Nullable{SpatialInertia{T}}
    frameDefinitions::Set{Transform3D{T}}

    # inertia undefined; can be used for the root of a kinematic tree
    function RigidBody(name::String)
        frame = CartesianFrame3D(name)
        new(name, Nullable{SpatialInertia{T}}(), Set([Transform3D{T}(frame)]))
    end

    # other bodies
    function RigidBody(name::String, inertia::SpatialInertia{T})
        new(name, Nullable(inertia), Set([Transform3D{T}(inertia.frame)]))
    end
end

eltype{T}(::Type{RigidBody{T}}) = T
eltype{T}(body::RigidBody{T}) = eltype(typeof(body))
RigidBody{T}(name::String, inertia::SpatialInertia{T}) = RigidBody{T}(name, inertia)
RigidBody{T}(inertia::SpatialInertia{T}) = RigidBody{T}(name(inertia.frame), inertia)
name(b::RigidBody) = b.name
show(io::IO, b::RigidBody) = print(io, "RigidBody: \"$(name(b))\"")
showcompact(io::IO, b::RigidBody) = print(io, "$(name(b))")
has_defined_inertia(b::RigidBody) = !isnull(b.inertia)
spatial_inertia(b::RigidBody) = get(b.inertia)
frame_definitions(body::RigidBody) = body.frameDefinitions
is_fixed_to_body(body::RigidBody, frame::CartesianFrame3D) = any((t) -> t.from == frame, frame_definitions(body))
default_frame(body::RigidBody) = first(frame_definitions(body)).to # allows standardization on a frame to reduce number of transformations required

function frame_definition(body::RigidBody, frame::CartesianFrame3D)
    for transform in body.frameDefinitions
        transform.from == frame && return transform
    end
    error("$frame not found among body fixed frame definitions for $body")
end

function fixed_transform(body::RigidBody, from::CartesianFrame3D, to::CartesianFrame3D)
    transform = frame_definition(body, from)
    if transform.to != to
        transform = inv(frame_definition(body, to)) * transform
    end
    transform
end

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
