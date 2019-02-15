const DEFAULT_GRAVITATIONAL_ACCELERATION = SVector(0.0, 0.0, -9.81)

"""
$(TYPEDEF)

A `Mechanism` represents an interconnection of rigid bodies and joints.
`Mechanism`s store the joint layout and inertia parameters, but no
state-dependent information.
"""
mutable struct Mechanism{T}
    graph::DirectedGraph{RigidBody{T}, Joint{T}}
    tree::SpanningTree{RigidBody{T}, Joint{T}}
    environment::ContactEnvironment{T}
    gravitational_acceleration::FreeVector3D{SVector{3, T}} # TODO: consider removing
    modcount::Int

    @doc """
    $(SIGNATURES)

    Create a new `Mechanism` containing only a root body, to which other bodies can
    be attached with joints.

    The `gravity` keyword argument can be used to set the gravitational acceleration
    (a 3-vector expressed in the `Mechanism`'s root frame). Default: `$(DEFAULT_GRAVITATIONAL_ACCELERATION)`.
    """ ->
    function Mechanism(root_body::RigidBody{T}; gravity::AbstractVector=DEFAULT_GRAVITATIONAL_ACCELERATION) where {T}
        graph = DirectedGraph{RigidBody{T}, Joint{T}}()
        add_vertex!(graph, root_body)
        tree = SpanningTree(graph, root_body)
        gravitational_acceleration = FreeVector3D(default_frame(root_body), convert(SVector{3, T}, gravity))
        environment = ContactEnvironment{T}()
        new{T}(graph, tree, environment, gravitational_acceleration, 0)
    end
end

Base.eltype(::Type{Mechanism{T}}) where {T} = T

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
path(mechanism::Mechanism, from::RigidBody, to::RigidBody) = TreePath(from, to, mechanism.tree)

has_loops(mechanism::Mechanism) = num_edges(mechanism.graph) > num_edges(mechanism.tree)

@inline modcount(mechanism::Mechanism) = mechanism.modcount
register_modification!(mechanism::Mechanism) = mechanism.modcount += 1

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
            show(IOContext(io, :compact => true), joint)
            print(io, ", predecessor: ")
            show(IOContext(io, :compact => true), predecessor(joint, mechanism))
            print(io, ", successor: ")
            show(IOContext(io, :compact => true), successor(joint, mechanism))
        end
    end
end

isroot(b::RigidBody{T}, mechanism::Mechanism{T}) where {T} = b == root_body(mechanism)
non_root_bodies(mechanism::Mechanism) = Base.unsafe_view(bodies(mechanism), 2 : length(bodies(mechanism)))
num_bodies(mechanism::Mechanism) = num_vertices(mechanism.graph)

"""
$(SIGNATURES)

Return the dimension of the joint configuration vector ``q``.
"""
num_positions(mechanism::Mechanism)::Int = mapreduce(num_positions, +, tree_joints(mechanism); init=0)

"""
$(SIGNATURES)

Return the dimension of the joint velocity vector ``v``.
"""
num_velocities(mechanism::Mechanism)::Int = mapreduce(num_velocities, +, tree_joints(mechanism); init=0)

"""
$(SIGNATURES)

Return the number of constraints imposed by the mechanism's non-tree joints (i.e., the number of rows of the constraint Jacobian).
"""
num_constraints(mechanism::Mechanism)::Int = mapreduce(num_constraints, +, non_tree_joints(mechanism); init=0)

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

function add_body_fixed_frame!(mechanism::Mechanism{T}, transform::Transform3D) where {T}
    add_frame!(body_fixed_frame_to_body(mechanism, transform.to), transform)
end

function canonicalize_frame_definitions!(mechanism::Mechanism{T}, body::RigidBody{T}) where {T}
    if !isroot(body, mechanism)
        change_default_frame!(body, frame_after(joint_to_parent(body, mechanism)))
    end
    for joint in in_joints(body, mechanism)
        set_joint_to_successor!(joint, frame_definition(body, frame_after(joint)))
    end
    for joint in out_joints(body, mechanism)
        set_joint_to_predecessor!(joint, frame_definition(body, frame_before(joint)))
    end
end

function canonicalize_frame_definitions!(mechanism::Mechanism)
    for body in bodies(mechanism)
        canonicalize_frame_definitions!(mechanism, body)
    end
end

function gravitational_spatial_acceleration(mechanism::Mechanism)
    frame = mechanism.gravitational_acceleration.frame
    SpatialAcceleration(frame, frame, frame, zero(SVector{3, eltype(mechanism)}), mechanism.gravitational_acceleration.v)
end

"""
$(SIGNATURES)

Return the `RigidBody` with the given name. Errors if there is no body with the given name,
or if there's more than one.
"""
findbody(mechanism::Mechanism, name::String) = findunique(b -> string(b) == name, bodies(mechanism))

"""
$(SIGNATURES)

Return the `Joint` with the given name. Errors if there is no joint with the given name,
or if there's more than one.
"""
findjoint(mechanism::Mechanism, name::String) = findunique(j -> string(j) == name, joints(mechanism))


"""
$(SIGNATURES)

Return the `RigidBody` with the given `BodyID`.
"""
function findbody(mechanism::Mechanism, id::BodyID)
    ret = bodies(mechanism)[id]
    BodyID(ret) == id || error() # should never happen
    ret
end

"""
$(SIGNATURES)

Return the `Joint` with the given `JointID`.
"""
function findjoint(mechanism::Mechanism, id::JointID)
    ret = joints(mechanism)[id]
    JointID(ret) == id || error() # should never happen
    ret
end
