"""
    attach!(mechanism, predecessor, joint, jointToPredecessor, successor, successorToJoint)

Attach `successor` to `predecessor` using `joint`.

See [`Joint`](@ref) for definitions of the terms successor and predecessor.

The `Transform3D`s `jointToPredecessor` and `successorToJoint` define where
`joint` is attached to each body. `jointToPredecessor` should define
`frame_before(joint)` with respect to any frame fixed to `predecessor`, and likewise
`successorToJoint` should define any frame fixed to `successor` with respect to
`frame_after(joint)`.

`predecessor` is required to already be among the bodies of the `Mechanism`.

If `successor` is not yet a part of the `Mechanism`, it will be added to the
`Mechanism`. Otherwise, the `joint` will be treated as a non-tree edge in the
`Mechanism`, effectively creating a loop constraint that will be enforced
using Lagrange multipliers (as opposed to using recursive algorithms).
"""
function attach!{T}(mechanism::Mechanism{T}, predecessor::RigidBody{T}, joint::Joint{T}, jointToPredecessor::Transform3D,
        successor::RigidBody{T}, successorToJoint::Transform3D = eye(STransform3D{T}, default_frame(successor), frame_after(joint)))
    @assert jointToPredecessor.from == frame_before(joint)
    @assert successorToJoint.to == frame_after(joint)

    # define where joint is attached on predecessor
    add_frame!(predecessor, jointToPredecessor)

    # define where child is attached to joint
    add_frame!(successor, inv(successorToJoint))

    if successor ∈ bodies(mechanism)
        add_edge!(mechanism.graph, predecessor, successor, joint)
    else
        add_edge!(mechanism.tree, predecessor, successor, joint)
        canonicalize_frame_definitions!(mechanism, successor)
    end
    mechanism
end

function _copyjoint!{T}(dest::Mechanism{T}, src::Mechanism{T}, srcjoint::Joint{T}, bodymap::Dict{RigidBody{T}, RigidBody{T}}, jointmap::Dict{Joint{T}, Joint{T}})
    srcpredecessor = source(srcjoint, src.graph)
    srcsuccessor = target(srcjoint, src.graph)

    joint_to_predecessor = fixed_transform(srcpredecessor, frame_before(srcjoint), default_frame(srcpredecessor))
    successor_to_joint = fixed_transform(srcsuccessor, default_frame(srcsuccessor), frame_after(srcjoint))

    destpredecessor = get!(() -> deepcopy(srcpredecessor), bodymap, srcpredecessor)
    destsuccessor = get!(() -> deepcopy(srcsuccessor), bodymap, srcsuccessor)
    destjoint = jointmap[srcjoint] = deepcopy(srcjoint)

    attach!(dest, destpredecessor, destjoint, joint_to_predecessor, destsuccessor, successor_to_joint)
end

"""
$(SIGNATURES)

Attach a copy of `childmechanism` to `mechanism`. Return mappings from the bodies and joints
of the `childmechanism` to the bodies and joints that were added to `mechanism`.

Essentially replaces the root body of a copy of `childmechanism` with `parentbody` (which
belongs to `mechanism`).

Note: gravitational acceleration for childmechanism is ignored.
"""
function attach!{T}(mechanism::Mechanism{T}, parentbody::RigidBody{T}, childmechanism::Mechanism{T},
        childroot_to_parent::Transform3D = eye(STransform3D{T}, default_frame(root_body(childmechanism)), default_frame(parentbody)))
    # FIXME: test with cycles

    @assert mechanism != childmechanism # infinite loop otherwise

    bodymap = Dict{RigidBody{T}, RigidBody{T}}()
    jointmap = Dict{Joint{T}, Joint{T}}()

    # Define where child root body is located w.r.t parent body and add frames that were attached to childroot to parentbody.
    childroot = root_body(childmechanism)
    add_frame!(parentbody, childroot_to_parent)
    for transform in frame_definitions(childroot)
        add_frame!(parentbody, transform)
    end
    canonicalize_frame_definitions!(mechanism, parentbody)
    bodymap[childroot] = parentbody

    # Insert childmechanism's non-root vertices and joints, starting with the tree joints (to preserve order).
    for joint in flatten((tree_joints(childmechanism), non_tree_joints(childmechanism)))
        _copyjoint!(mechanism, childmechanism, joint, bodymap, jointmap)
    end
    bodymap, jointmap
end


"""
$(SIGNATURES)

Create a new `Mechanism` from the subtree of `mechanism` rooted at `submechanismroot`.

Also return mappings from the bodies and joints of the input mechanism to the
bodies and joints of the submechanism.

Any non-tree joint in `mechanism` will appear in the returned `Mechanism` if and
only if both its successor and its predecessor are part of the subtree.
"""
function submechanism{T}(mechanism::Mechanism{T}, submechanismroot::RigidBody{T})
    # FIXME: test with cycles

    bodymap = Dict{RigidBody{T}, RigidBody{T}}()
    jointmap = Dict{Joint{T}, Joint{T}}()

    # Create Mechanism
    root = bodymap[submechanismroot] = deepcopy(submechanismroot)
    ret = Mechanism{T}(root; gravity = mechanism.gravitationalAcceleration.v)

    # Add tree joints, preserving order in input mechanism.
    for joint in tree_joints(mechanism) # assumes toposort
        if haskey(bodymap, predecessor(joint, mechanism))
            _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
        end
    end

    # Add non-tree joints.
    for joint in non_tree_joints(mechanism)
        if haskey(bodymap, predecessor(joint, mechanism)) && haskey(bodymap, successor(joint, mechanism))
            _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
        end
    end

    ret, bodymap, jointmap
end

"""
$(SIGNATURES)

Reconstruct the mechanism's spanning tree.
"""
function rebuild_spanning_tree!(mechanism::Mechanism, next_edge = first #= breadth first =#)
    mechanism.tree = SpanningTree(mechanism.graph, root_body(mechanism), next_edge)
    canonicalize_frame_definitions!(mechanism)
end

"""
$(SIGNATURES)

Remove a joint from the mechanism. Rebuilds the spanning tree if the joint is
part of the current spanning tree.
"""
function remove_joint!(mechanism::Mechanism, joint::Joint, spanning_tree_next_edge = first #= breadth first =#)
    istreejoint = joint ∈ tree_joints(mechanism)
    remove_edge!(mechanism.graph, joint)
    istreejoint && rebuild_spanning_tree!(mechanism, spanning_tree_next_edge)
end

Base.@deprecate(
reattach!{T}(mechanism::Mechanism{T}, oldSubtreeRootBody::RigidBody{T},
    parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D,
    newSubtreeRootBody::RigidBody{T}, newSubTreeRootBodyToJoint::Transform3D = eye(STransform3D{T}, default_frame(newSubtreeRootBody), frame_after(joint))),
    begin
        attach!(mechanism, parentBody, joint, jointToParent, newSubtreeRootBody, newSubTreeRootBodyToJoint)
        remove_joint!(mechanism, joint_to_parent(oldSubtreeRootBody, mechanism))
    end)

Base.@deprecate remove_fixed_joints!(mechanism::Mechanism) remove_fixed_tree_joints!(mechanism)

"""
$(SIGNATURES)

Remove any fixed joints present as tree edges in `mechanism` by merging the
rigid bodies that these fixed joints join together into bodies with equivalent
inertial properties. Return the fixed joints that were removed.
"""
function remove_fixed_tree_joints!(mechanism::Mechanism)
    # FIXME: test with cycles
    T = eltype(mechanism)
    graph = mechanism.graph

    # Update graph.
    fixedjoints = filter(j -> isa(j.jointType, Fixed), tree_joints(mechanism))
    newtreejoints = setdiff(tree_joints(mechanism), fixedjoints)
    for fixedjoint in fixedjoints
        pred = source(fixedjoint, graph)
        succ = target(fixedjoint, graph)

        # Add identity joint transform as a body-fixed frame definition.
        jointtransform = eye(STransform3D{T}, frame_after(fixedjoint), frame_before(fixedjoint))
        add_frame!(pred, jointtransform)

        # Migrate body fixed frames to parent body.
        for tf in frame_definitions(succ)
            add_frame!(pred, tf)
        end

        # Add inertia to parent body.
        if has_defined_inertia(pred)
            inertia = spatial_inertia(succ)
            parentinertia = spatial_inertia(pred)
            toparent = fixed_transform(pred, inertia.frame, parentinertia.frame)
            spatial_inertia!(pred, parentinertia + transform(inertia, toparent))
        end

        # Merge vertex into parent.
        for joint in copy(in_edges(succ, graph))
            if joint == fixedjoint
                remove_edge!(graph, joint)
            else
                rewire!(graph, joint, source(joint, graph), pred)
            end
        end
        for joint in copy(out_edges(succ, graph))
            rewire!(graph, joint, pred, target(joint, graph))
        end
        remove_vertex!(mechanism.graph, succ)
    end

    # Recompute spanning tree (preserves order for non-fixed joints)
    mechanism.tree = SpanningTree(graph, root_body(mechanism), newtreejoints)

    # Recanonicalize frames
    canonicalize_frame_definitions!(mechanism)

    fixedjoints
end

# TODO: remove floating_non_tree_joints

"""
$(SIGNATURES)

Return a dynamically equivalent `Mechanism`, but with a flat tree structure
with all bodies attached to the root body with a quaternion floating joint, and
with the 'tree edge' joints of the input `Mechanism` transformed into non-tree
edge joints (a constraint enforced using Lagrange multipliers in `dynamics!`).
In addition, return:
* a mapping from bodies in the maximal-coordinate `Mechanism` to their floating joints.
* a mapping from bodies in the input `Mechanism` to bodies in the returned `Mechanism`
* a mapping from joints in the input `Mechanism` to joints in the returned `Mechanism`
"""
function maximal_coordinates(mechanism::Mechanism)
    T = eltype(mechanism)

    # Body and joint mapping.
    bodymap = Dict{RigidBody{T}, RigidBody{T}}()
    jointmap = Dict{Joint{T}, Joint{T}}()

    # Copy root.
    root = bodymap[root_body(mechanism)] = deepcopy(root_body(mechanism))
    ret = Mechanism(root, gravity = mechanism.gravitationalAcceleration.v)

    # Copy non-root bodies and attach them to the root with a floating joint.
    newfloatingjoints = Dict{RigidBody{T}, Joint{T}}()
    for srcbody in non_root_bodies(mechanism)
        framebefore = default_frame(root)
        frameafter = default_frame(srcbody)
        body = bodymap[srcbody] = deepcopy(srcbody)
        floatingjoint = newfloatingjoints[body] = Joint(name(body), framebefore, frameafter, QuaternionFloating{T}())
        attach!(ret, root, floatingjoint, eye(STransform3D{T}, framebefore), body, eye(STransform3D{T}, frameafter))
    end

    # Copy input Mechanism's joints.
    for joint in flatten((tree_joints(mechanism), non_tree_joints(mechanism)))
        _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
    end

    ret, newfloatingjoints, bodymap, jointmap
end

add_environment_primitive!(mechanism::Mechanism, halfspace::HalfSpace3D) = push!(mechanism.environment, halfspace)
"""
$(SIGNATURES)

Create a random tree `Mechanism` with the given joint types. Each new body is
attached to a parent selected using the `parentselector` function.
"""
function rand_tree_mechanism{T}(::Type{T}, parentselector::Function, jointTypes...)
    parentbody = RigidBody{T}("world")
    mechanism = Mechanism(parentbody)
    for i = 1 : length(jointTypes)
        @assert jointTypes[i] <: JointType{T}
        joint = Joint("joint$i", rand(jointTypes[i]))
        jointToParentBody = rand(STransform3D{T}, frame_before(joint), default_frame(parentbody))
        body = RigidBody(rand(SpatialInertia{T}, CartesianFrame3D("body$i")))
        body_to_joint = eye(STransform3D{T}, default_frame(body), frame_after(joint))
        attach!(mechanism, parentbody, joint, jointToParentBody, body, body_to_joint)
        parentbody = parentselector(mechanism)
    end
    return mechanism
end

"""
$(SIGNATURES)

Create a random chain `Mechanism` with the given joint types.
"""
rand_chain_mechanism{T}(t::Type{T}, jointTypes...) = rand_tree_mechanism(t, mechanism::Mechanism -> last(bodies(mechanism)), jointTypes...)

"""
$(SIGNATURES)

Create a random tree `Mechanism`.
"""
rand_tree_mechanism{T}(t::Type{T}, jointTypes...) = rand_tree_mechanism(t, mechanism::Mechanism -> rand(bodies(mechanism)), jointTypes...)

"""
$(SIGNATURES)

Create a random tree `Mechanism`, with a quaternion floating
joint as the first joint (between the root body and the first non-root body).
"""
function rand_floating_tree_mechanism{T}(t::Type{T}, nonFloatingJointTypes...)
    parentselector = (mechanism::Mechanism) -> begin
        only_root = length(bodies(mechanism)) == 1
        only_root ? root_body(mechanism) : rand(collect(non_root_bodies(mechanism)))
    end
    rand_tree_mechanism(t, parentselector, [QuaternionFloating{T}; nonFloatingJointTypes...]...)
end
