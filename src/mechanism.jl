"""
$(TYPEDEF)

A `Mechanism` represents an interconnection of rigid bodies and joints.
`Mechanism`s store the joint layout and inertia parameters, but no
state-dependent information.
"""
type Mechanism{T<:Number}
    graph::DirectedGraph{RigidBody{T}, GenericJoint{T}}
    tree::SpanningTree{RigidBody{T}, GenericJoint{T}}
    environment::ContactEnvironment{T}
    gravitationalAcceleration::FreeVector3D{SVector{3, T}} # TODO: consider removing

    function (::Type{Mechanism{T}}){T<:Number}(rootBody::RigidBody{T}; gravity::SVector{3, T} = SVector(zero(T), zero(T), T(-9.81)))
        graph = DirectedGraph{RigidBody{T}, GenericJoint{T}}()
        add_vertex!(graph, rootBody)
        tree = SpanningTree(graph, rootBody)
        gravitationalAcceleration = FreeVector3D(default_frame(rootBody), gravity)
        environment = ContactEnvironment{T}()
        new{T}(graph, tree, environment, gravitationalAcceleration)
    end
end

"""
$(SIGNATURES)

Create a new `Mechanism` containing only a root body, to which other bodies can
be attached with joints.
"""
Mechanism{T}(rootBody::RigidBody{T}; kwargs...) = Mechanism{T}(rootBody; kwargs...)
Base.eltype{T}(::Mechanism{T}) = T

"""
$(SIGNATURES)

Return the `Joint`s that are part of the `Mechanism` as an iterable collection.
"""
joints(mechanism::Mechanism) = edges(mechanism.graph)

"""
$(SIGNATURES)

Return the `Joint`s that are part of the `Mechanism`'s spanning tree as an iterable collection.
"""
tree_joints(mechanism::Mechanism) = edges(mechanism.tree)

"""
$(SIGNATURES)

Return the `Joint`s that are not part of the `Mechanism`'s spanning tree as an iterable collection.
"""
non_tree_joints(mechanism::Mechanism) = setdiff(edges(mechanism.graph), edges(mechanism.tree))

"""
$(SIGNATURES)

Return the `RigidBody`s that are part of the `Mechanism` as an iterable collection.
"""
bodies(mechanism::Mechanism) = vertices(mechanism.graph)

"""
$(SIGNATURES)

Return the root (stationary 'world') body of the `Mechanism`.
"""
root_body(mechanism::Mechanism) = first(bodies(mechanism))

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
RigidBodyDynamics.path(mechanism::Mechanism, from::RigidBody, to::RigidBody) = Graphs.path(from, to, mechanism.tree)

function Base.show(io::IO, mechanism::Mechanism)
    println(io, "Spanning tree:")
    print(io, mechanism.tree)
    nontreejoints = non_tree_joints(mechanism)
    println(io)
    if isempty(nontreejoints)
        print(io, "No non-tree joints.")
    else
        print(io, "Non-tree joints:")
        for joint in nontreejoints
            println(io)
            showcompact(io, joint)
            print(io, ", predecessor: ")
            showcompact(io, predecessor(joint, mechanism))
            print(io, ", successor: ")
            showcompact(io, successor(joint, mechanism))
        end
    end
end
Base.@deprecate isroot{T}(mechanism::Mechanism{T}, b::RigidBody{T}) isroot(b, mechanism)
isroot{T}(b::RigidBody{T}, mechanism::Mechanism{T}) = b == root_body(mechanism)
non_root_bodies{T}(mechanism::Mechanism{T}) = (body for body in bodies(mechanism) if !isroot(body, mechanism))
num_bodies(mechanism::Mechanism) = num_vertices(mechanism.graph)

"""
$(SIGNATURES)

Return the dimension of the joint configuration vector ``q``.
"""
num_positions(mechanism::Mechanism) = mapreduce(num_positions, +, 0, tree_joints(mechanism))

"""
$(SIGNATURES)

Return the dimension of the joint velocity vector ``v``.
"""
num_velocities(mechanism::Mechanism) = mapreduce(num_velocities, +, 0, tree_joints(mechanism))

"""
$(SIGNATURES)

Return the dimension of the vector of additional states ``s`` (used for stateful contact models).
"""
function num_additional_states(mechanism::Mechanism)
    num_states_per_environment_element = 0
    for body in bodies(mechanism), point in contact_points(body)
        num_states_per_environment_element += num_states(contact_model(point))
    end
    num_states_per_environment_element * length(mechanism.environment)
end

"""
$(SIGNATURES)

Return the `RigidBody` to which `frame` is attached.

Note: this function is linear in the number of bodies and is not meant to be
called in tight loops.
"""
function body_fixed_frame_to_body(mechanism::Mechanism, frame::CartesianFrame3D)
    for body in bodies(mechanism)
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

Note: this function is linear in the number of bodies and is not meant to be
called in tight loops.

See also [`default_frame`](@ref), [`frame_definition`](@ref).
"""
function body_fixed_frame_definition(mechanism::Mechanism, frame::CartesianFrame3D)
    frame_definition(body_fixed_frame_to_body(mechanism, frame), frame)
end

"""
$(SIGNATURES)

Return the transform from `CartesianFrame3D` `from` to `to`, both of which are
rigidly attached to the same `RigidBody`.

Note: this function is linear in the number of bodies and is not meant to be
called in tight loops.
"""
function fixed_transform(mechanism::Mechanism, from::CartesianFrame3D, to::CartesianFrame3D)
    fixed_transform(body_fixed_frame_to_body(mechanism, from), from, to)
end

"""
$(SIGNATURES)

Return the body 'before' the joint, i.e. the 'tail' of the joint interpreted as an
arrow in the `Mechanism`'s kinematic graph.

See [`Joint`](@ref).
"""
predecessor(joint::Joint, mechanism::Mechanism) = source(joint, mechanism.graph)

"""
$(SIGNATURES)

Return the body 'after' the joint, i.e. the 'head' of the joint interpreted as an
arrow in the `Mechanism`'s kinematic graph.

See [`Joint`](@ref).
"""
successor(joint::Joint, mechanism::Mechanism) = target(joint, mechanism.graph)

"""
$(SIGNATURES)

Return the joints that have `body` as their [`predecessor`](@ref).
"""
out_joints(body::RigidBody, mechanism::Mechanism) = out_edges(body, mechanism.graph)

"""
$(SIGNATURES)

Return the joints that have `body` as their [`successor`](@ref).
"""
in_joints(body::RigidBody, mechanism::Mechanism) = in_edges(body, mechanism.graph)

"""
$(SIGNATURES)

Return the joints that are part of the mechanism's kinematic tree and have
`body` as their predecessor.
"""
joints_to_children(body::RigidBody, mechanism::Mechanism) = edges_to_children(body, mechanism.tree)

"""
$(SIGNATURES)

Return the joint that is part of the mechanism's kinematic tree and has
`body` as its successor.
"""
joint_to_parent(body::RigidBody, mechanism::Mechanism) = edge_to_parent(body, mechanism.tree)


Base.@deprecate add_body_fixed_frame!{T}(mechanism::Mechanism{T}, body::RigidBody{T}, transform::Transform3D) add_frame!(body, transform)
function add_body_fixed_frame!{T}(mechanism::Mechanism{T}, transform::Transform3D)
    add_frame!(body_fixed_frame_to_body(mechanism, transform.to), transform)
end

function canonicalize_frame_definitions!{T}(mechanism::Mechanism{T}, body::RigidBody{T})
    if !isroot(body, mechanism)
        joint = joint_to_parent(body, mechanism)
        change_default_frame!(body, frame_after(joint))
    end
end

function canonicalize_frame_definitions!(mechanism::Mechanism)
    for body in bodies(mechanism)
        canonicalize_frame_definitions!(mechanism, body)
    end
end

function gravitational_spatial_acceleration(mechanism::Mechanism)
    frame = mechanism.gravitationalAcceleration.frame
    SpatialAcceleration(frame, frame, frame, zeros(SVector{3, eltype(mechanism)}), mechanism.gravitationalAcceleration.v)
end

tree_index(joint::Joint, mechanism::Mechanism) = Graphs.tree_index(joint, mechanism.tree)
tree_index(body::RigidBody, mechanism::Mechanism) = Graphs.tree_index(body, mechanism.tree)

findbody(mechanism::Mechanism, name::String) = findunique(b -> b.name == name, bodies(mechanism))
findjoint(mechanism::Mechanism, name::String) = findunique(j -> j.name == name, joints(mechanism))
