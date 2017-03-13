"""
    attach!(mechanism, predecessor, joint, jointToPredecessor, successor, successorToJoint)

Attach `successor` to `predecessor` using `joint`.

See [`Joint`](@ref) for definitions of the terms successor and predecessor.

The `Transform3D`s `jointToPredecessor` and `successorToJoint` define where
`joint` is attached to each body. `jointToPredecessor` should define
`joint.frameBefore` with respect to any frame fixed to `predecessor`, and likewise
`successorToJoint` should define any frame fixed to `successor` with respect to
`joint.frameAfter`.

`predecessor` is required to already be among the bodies of the `Mechanism`.

If `successor` is not yet a part of the `Mechanism`, it will be added to the
`Mechanism`. Otherwise, the `joint` will be treated as a non-tree edge in the
`Mechanism`, effectively creating a loop constraint that will be enforced
using Lagrange multipliers (as opposed to using recursive algorithms).
"""
function attach!{T}(mechanism::Mechanism{T}, predecessor::RigidBody{T}, joint::Joint, jointToPredecessor::Transform3D{T},
        successor::RigidBody{T}, successorToJoint::Transform3D{T} = Transform3D{T}(default_frame(successor), joint.frameAfter))
    # TODO: check that jointToPredecessor.from and successorToJoint.to match joint.

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

    joint_to_predecessor = fixed_transform(srcpredecessor, srcjoint.frameBefore, default_frame(srcpredecessor))
    successor_to_joint = fixed_transform(srcsuccessor, default_frame(srcsuccessor), srcjoint.frameAfter)

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
        childroot_to_parent::Transform3D{T} = Transform3D(T, default_frame(root_body(childmechanism)), default_frame(parentbody)))
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
        if haskey(bodymap, source(joint, mechanism.graph))
            _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
        end
    end

    # Add non-tree joints.
    for joint in non_tree_joints(mechanism)
        if haskey(bodymap, source(joint, mechanism.graph)) && haskey(bodymap, target(joint, mechanism.graph))
            _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
        end
    end

    ret, bodymap, jointmap
end

# FIXME: reimplement this functionality in a simplified way
# """
# $(SIGNATURES)
#
# Detach the subtree rooted at `oldSubtreeRootBody`, reroot it so that
# `newSubtreeRootBody` is the new root, and then attach `newSubtreeRootBody`
# to `parentBody` using `joint`.
#
# Currently doesn't support `Mechanism`s with cycles.
# """
# function reattach!{T}(mechanism::Mechanism{T}, oldSubtreeRootBody::RigidBody{T},
#         parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T},
#         newSubtreeRootBody::RigidBody{T}, newSubTreeRootBodyToJoint::Transform3D{T} = Transform3D{T}(default_frame(newSubtreeRootBody), joint.frameAfter))
#     # TODO: add option to prune frames related to old joints
#     check_no_cycles(mechanism)
#
#     newSubtreeRoot = findfirst(tree(mechanism), newSubtreeRootBody)
#     oldSubtreeRoot = findfirst(tree(mechanism), oldSubtreeRootBody)
#     parentVertex = findfirst(tree(mechanism), parentBody)
#     @assert newSubtreeRoot ∈ toposort(oldSubtreeRoot)
#     @assert parentVertex ∉ toposort(oldSubtreeRoot)
#
#     # detach oldSubtreeRoot
#     detach!(oldSubtreeRoot)
#
#     # reroot
#     flippedJoints = Dict{Joint{T}, Joint{T}}()
#     flipDirectionFunction = joint -> begin
#         flipped = Joint(joint.name * "_flipped", flip_direction(joint.jointType))
#         flippedJoints[joint] = flipped
#         flipped
#     end
#     subtreeRerooted = reroot(newSubtreeRoot, flipDirectionFunction)
#
#     # attach newSubtreeRoot using joint
#     insert!(parentVertex, subtreeRerooted, joint)
#     mechanism.toposortedTree = toposort(tree(mechanism))
#
#     # define frames related to new joint
#     add_frame!(parentBody, jointToParent)
#     add_frame!(newSubtreeRootBody, inv(newSubTreeRootBodyToJoint))
#
#     # define identities between new frames and old frames and recanonicalize frame definitions
#     for (oldJoint, newJoint) in flippedJoints
#         add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameBefore, oldJoint.frameAfter))
#         add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameAfter, oldJoint.frameBefore))
#     end
#     canonicalize_frame_definitions!(mechanism)
#
#     flippedJoints
# end

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
    for fixedjoint in fixedjoints
        predecessor = source(fixedjoint, graph)
        successor = target(fixedjoint, graph)

        # Add identity joint transform as a body-fixed frame definition.
        jointtransform = Transform3D{T}(fixedjoint.frameAfter, fixedjoint.frameBefore)
        add_frame!(predecessor, jointtransform)

        # Migrate body fixed frames to parent body.
        for tf in frame_definitions(successor)
            add_frame!(predecessor, tf)
        end

        # Add inertia to parent body.
        if has_defined_inertia(predecessor)
            inertia = spatial_inertia(successor)
            parentinertia = spatial_inertia(predecessor)
            toparent = fixed_transform(predecessor, inertia.frame, parentinertia.frame)
            spatial_inertia!(predecessor, parentinertia + transform(inertia, toparent))
        end

        # Merge vertex into parent.
        for joint in in_edges(successor, graph)
            if joint == fixedjoint
                remove_edge!(graph, joint)
            else
                rewire!(graph, joint, source(joint, graph), predecessor)
            end
        end
        for joint in out_edges(successor, graph)
            rewire!(graph, joint, predecessor, target(joint, graph))
        end
        remove_vertex!(mechanism.graph, successor)
    end

    # Recompute spanning tree (preserves order for non-fixed joints)
    mechanism.tree = SpanningTree(graph, setdiff(tree_joints(mechanism), fixedjoints))

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
        attach!(ret, root, floatingjoint, Transform3D(T, framebefore), body, Transform3D(T, frameafter))
    end

    # Copy input Mechanism's joints.
    for joint in flatten((tree_joints(mechanism), non_tree_joints(mechanism)))
        _copyjoint!(ret, mechanism, joint, bodymap, jointmap)
    end

    ret, newfloatingjoints, bodymap, jointmap
end

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
        jointToParentBody = rand(Transform3D{T}, joint.frameBefore, default_frame(parentbody))
        body = RigidBody(rand(SpatialInertia{T}, CartesianFrame3D("body$i")))
        bodyToJoint = Transform3D{T}(default_frame(body), joint.frameAfter)
        attach!(mechanism, parentbody, joint, jointToParentBody, body, bodyToJoint)
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
    rand_mechanism(t, parentselector, [QuaternionFloating{T}; nonFloatingJointTypes...]...)
end
