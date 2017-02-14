type NonTreeEdge{T}
    joint::Joint{T}
    predecessor::RigidBody{T}
    successor::RigidBody{T}
end

"""
$(TYPEDEF)

A `Mechanism` represents an interconnection of rigid bodies and joints.
`Mechanism`s store the joint layout and inertia parameters, but no
state-dependent information.
"""
type Mechanism{T<:Number}
    toposortedTree::Vector{TreeVertex{RigidBody{T}, Joint{T}}} # TODO: consider replacing with just the root vertex after creating iterator
    nonTreeEdges::Vector{NonTreeEdge{T}}
    gravitationalAcceleration::FreeVector3D{SVector{3, T}} # TODO: consider removing

    function Mechanism(rootBody::RigidBody{T}; gravity::SVector{3, T} = SVector(zero(T), zero(T), T(-9.81)))
        tree = Tree{RigidBody{T}, Joint{T}}(rootBody)
        nonTreeEdges = Vector{NonTreeEdge{T}}()
        gravitationalAcceleration = FreeVector3D(default_frame(rootBody), gravity)
        new(toposort(tree), nonTreeEdges, gravitationalAcceleration)
    end
end

"""
$(SIGNATURES)

Create a new `Mechanism` containing only a root body, to which other bodies can
be attached with joints.
"""
Mechanism{T}(rootBody::RigidBody{T}; kwargs...) = Mechanism{T}(rootBody; kwargs...)
Base.eltype{T}(::Mechanism{T}) = T
root_vertex(mechanism::Mechanism) = mechanism.toposortedTree[1]
non_root_vertices(mechanism::Mechanism) = view(mechanism.toposortedTree, 2 : length(mechanism.toposortedTree)) # TODO: allocates
tree(mechanism::Mechanism) = mechanism.toposortedTree[1]

"""
$(SIGNATURES)

Return the root (stationary 'world') body of the `Mechanism`.
"""
root_body(mechanism::Mechanism) = vertex_data(root_vertex(mechanism))

"""
$(SIGNATURES)

Return the default frame of the root body.
"""
root_frame(mechanism::Mechanism) = default_frame(root_body(mechanism))

"""
$(SIGNATURES)

Return the path from rigid body `from` to `to` along edges of the `Mechanism`'s
kinematic tree.
"""
path(mechanism::Mechanism, from::RigidBody, to::RigidBody) = path(findfirst(tree(mechanism), from), findfirst(tree(mechanism), to))
Base.show(io::IO, mechanism::Mechanism) = print(io, mechanism.toposortedTree[1])
isinertial(mechanism::Mechanism, frame::CartesianFrame3D) = is_fixed_to_body(frame, root_body(mechanism))
isroot{T}(mechanism::Mechanism{T}, b::RigidBody{T}) = b == root_body(mechanism)
non_root_bodies{T}(mechanism::Mechanism{T}) = (vertex_data(vertex) for vertex in non_root_vertices(mechanism))
num_bodies(mechanism::Mechanism) = length(mechanism.toposortedTree)

"""
$(SIGNATURES)

Return the `Joint`s that are part of the `Mechanism` as an iterable collection.
"""
joints(mechanism::Mechanism) = (edge_to_parent_data(vertex) for vertex in non_root_vertices(mechanism))

"""
$(SIGNATURES)

Return the `RigidBody`s that are part of the `Mechanism` as an iterable collection.
"""
bodies{T}(mechanism::Mechanism{T}) = (vertex_data(vertex) for vertex in mechanism.toposortedTree)


"""
$(SIGNATURES)

Return the dimension of the joint configuration vector ``q``.
"""
num_positions(mechanism::Mechanism) = num_positions(joints(mechanism))

"""
$(SIGNATURES)

Return the dimension of the joint velocity vector ``v``.
"""
num_velocities(mechanism::Mechanism) = num_velocities(joints(mechanism))

"""
$(SIGNATURES)

Return the `RigidBody` to which `frame` is attached.
"""
function body_fixed_frame_to_body(mechanism::Mechanism, frame::CartesianFrame3D)
    # linear in number of bodies and number of frames; not meant to be super fast
    for vertex in mechanism.toposortedTree
        body = vertex_data(vertex)
        if is_fixed_to_body(body, frame)
            return body
        end
    end
    error("Unable to determine to what body $(frame) is attached")
end

"""
$(SIGNATURES)

Return the definition of body-fixed frame `frame`, i.e., the `Transform3D` from
`frame` to the default frame of the body to which it is attached.

See also [`default_frame`](@ref), [`frame_definition`](@ref).
"""
function body_fixed_frame_definition(mechanism::Mechanism, frame::CartesianFrame3D)
    frame_definition(body_fixed_frame_to_body(mechanism, frame), frame)
end

"""
$(SIGNATURES)

Return the transform from `CartesianFrame3D` `from` to `to`, both of which are
rigidly attached to the same `RigidBody`.
"""
function fixed_transform(mechanism::Mechanism, from::CartesianFrame3D, to::CartesianFrame3D)
    fixed_transform(body_fixed_frame_to_body(mechanism, from), from, to)
end

Base.@deprecate add_body_fixed_frame!{T}(mechanism::Mechanism{T}, body::RigidBody{T}, transform::Transform3D{T}) add_frame!(body, transform)
function add_body_fixed_frame!{T}(mechanism::Mechanism{T}, transform::Transform3D{T})
    add_frame!(body_fixed_frame_to_body(mechanism, transform.to), transform)
end

function canonicalize_frame_definitions!{T}(mechanism::Mechanism{T}, vertex::TreeVertex{RigidBody{T}, Joint{T}})
    if !isroot(vertex)
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        parentBody = vertex_data(parent(vertex))
        newDefaultFrame = joint.frameAfter
        change_default_frame!(body, newDefaultFrame)
    end
end

function canonicalize_frame_definitions!(mechanism::Mechanism)
    for vertex in non_root_vertices(mechanism)
        canonicalize_frame_definitions!(mechanism, vertex)
    end
end

function gravitational_spatial_acceleration(mechanism::Mechanism)
    frame = mechanism.gravitationalAcceleration.frame
    SpatialAcceleration(frame, frame, frame, zeros(SVector{3, eltype(mechanism)}), mechanism.gravitationalAcceleration.v)
end

function constraint_jacobian_structure(mechanism::Mechanism)
    # columns correspond to non-tree edges, rows correspond to tree joints
    ret = spzeros(Int64, length(mechanism.toposortedTree), length(mechanism.nonTreeEdges))
    for (col, nonTreeEdge) in enumerate(mechanism.nonTreeEdges)
        predecessorVertex = findfirst(v -> vertex_data(v) == nonTreeEdge.predecessor, tree(mechanism))
        successorVertex = findfirst(v -> vertex_data(v) == nonTreeEdge.successor, tree(mechanism))
        for (vertex, sign) in Dict(successorVertex => 1, predecessorVertex => -1)
            while !isroot(vertex)
                row = findfirst(mechanism.toposortedTree, vertex)
                ret[row, col] += sign
                vertex = parent(vertex)
            end
        end
    end
    dropzeros!(ret)
end
