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
        push!(mechanism.nonTreeEdges, NonTreeEdge(joint, predecessor, successor))
    else
        vertex = insert!(tree(mechanism), successor, joint, predecessor)
        mechanism.toposortedTree = toposort(tree(mechanism))
        canonicalize_frame_definitions!(mechanism, vertex)
    end
    mechanism
end

check_no_cycles(mechanism::Mechanism) = (length(mechanism.nonTreeEdges) == 0 || error("Mechanisms with cycles not yet supported."))

"""
$(SIGNATURES)

Attach `childMechanism` to `mechanism`.

Essentially replaces the root body of `childMechanism` with `parentBody` (which
belongs to `mechanism`).

Currently doesn't support `Mechanism`s with cycles.
"""
function attach!{T}(mechanism::Mechanism{T}, parentBody::RigidBody{T}, childMechanism::Mechanism{T})
    check_no_cycles(mechanism)
    check_no_cycles(childMechanism)

    # note: gravitational acceleration for childMechanism is ignored.
    parentVertex = findfirst(tree(mechanism), parentBody)
    childRootVertex = root_vertex(childMechanism)
    childRootBody = vertex_data(childRootVertex)

    # define where child root body is located w.r.t parent body
    # and add frames that were attached to childRootBody to parentBody
    add_frame!(parentBody, Transform3D{T}(default_frame(childRootBody), default_frame(parentBody))) # TODO: add optional function argument
    for transform in frame_definitions(childRootBody)
        add_frame!(parentBody, transform)
    end
    canonicalize_frame_definitions!(mechanism, parentVertex)

    # merge trees
    for child in children(childRootVertex)
        vertex = insert_subtree!(parentVertex, child)
        canonicalize_frame_definitions!(mechanism, vertex)
    end

    mechanism.toposortedTree = toposort(tree(mechanism))

    mechanism
end

"""
$(SIGNATURES)

Create a `Mechanism` from the subtree of `mechanism` rooted at `submechanismRootBody`.

Currently doesn't support `Mechanism`s with cycles.
"""
function submechanism{T}(mechanism::Mechanism{T}, submechanismRootBody::RigidBody{T})
    check_no_cycles(mechanism)

    # Create mechanism and set up tree
    ret = Mechanism{T}(submechanismRootBody; gravity = mechanism.gravitationalAcceleration.v)
    for child in children(findfirst(tree(mechanism), submechanismRootBody))
        insert_subtree!(root_vertex(ret), child)
    end
    ret.toposortedTree = toposort(tree(ret))
    canonicalize_frame_definitions!(ret)
    ret
end

"""
$(SIGNATURES)

Detach the subtree rooted at `oldSubtreeRootBody`, reroot it so that
`newSubtreeRootBody` is the new root, and then attach `newSubtreeRootBody`
to `parentBody` using `joint`.

Currently doesn't support `Mechanism`s with cycles.
"""
function reattach!{T}(mechanism::Mechanism{T}, oldSubtreeRootBody::RigidBody{T},
        parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T},
        newSubtreeRootBody::RigidBody{T}, newSubTreeRootBodyToJoint::Transform3D{T} = Transform3D{T}(default_frame(newSubtreeRootBody), joint.frameAfter))
    # TODO: add option to prune frames related to old joints
    check_no_cycles(mechanism)

    newSubtreeRoot = findfirst(tree(mechanism), newSubtreeRootBody)
    oldSubtreeRoot = findfirst(tree(mechanism), oldSubtreeRootBody)
    parentVertex = findfirst(tree(mechanism), parentBody)
    @assert newSubtreeRoot ∈ toposort(oldSubtreeRoot)
    @assert parentVertex ∉ toposort(oldSubtreeRoot)

    # detach oldSubtreeRoot
    detach!(oldSubtreeRoot)

    # reroot
    flippedJoints = Dict{Joint{T}, Joint{T}}()
    flipDirectionFunction = joint -> begin
        flipped = Joint(joint.name * "_flipped", flip_direction(joint.jointType))
        flippedJoints[joint] = flipped
        flipped
    end
    subtreeRerooted = reroot(newSubtreeRoot, flipDirectionFunction)

    # attach newSubtreeRoot using joint
    insert!(parentVertex, subtreeRerooted, joint)
    mechanism.toposortedTree = toposort(tree(mechanism))

    # define frames related to new joint
    add_frame!(parentBody, jointToParent)
    add_frame!(newSubtreeRootBody, inv(newSubTreeRootBodyToJoint))

    # define identities between new frames and old frames and recanonicalize frame definitions
    for (oldJoint, newJoint) in flippedJoints
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameBefore, oldJoint.frameAfter))
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameAfter, oldJoint.frameBefore))
    end
    canonicalize_frame_definitions!(mechanism)

    flippedJoints
end

"""
$(SIGNATURES)

Remove any fixed joints present as tree edges in `mechanism` by merging the
rigid bodies that these fixed joints join together into bodies with equivalent
inertial properties.

Currently doesn't support `Mechanism`s with cycles.
"""
function remove_fixed_joints!(mechanism::Mechanism)
    check_no_cycles(mechanism)
    T = eltype(mechanism)
    for vertex in copy(mechanism.toposortedTree)
        if !isroot(vertex)
            body = vertex_data(vertex)
            joint = edge_to_parent_data(vertex)
            parentVertex = parent(vertex)
            parentBody = vertex_data(parentVertex)
            if isa(joint.jointType, Fixed)
                # add identity joint transform as a body-fixed frame definition
                jointTransform = Transform3D{T}(joint.frameAfter, joint.frameBefore)
                add_frame!(parentBody, jointTransform)

                # migrate body fixed frames to parent body
                for tf in frame_definitions(body)
                    add_frame!(parentBody, tf)
                end

                # add inertia to parent body
                if has_defined_inertia(parentBody)
                    inertia = spatial_inertia(body)
                    parentInertia = spatial_inertia(parentBody)
                    toParent = fixed_transform(parentBody, inertia.frame, parentInertia.frame)
                    parentBody.inertia = parentInertia + transform(inertia, toParent)
                end

                # merge vertex into parent
                formerChildren = children(vertex)
                merge_into_parent!(vertex)

                # recanonicalize children since their new parent's default frame may be different
                for child in formerChildren
                    canonicalize_frame_definitions!(mechanism, child)
                end
            end
        end
    end
    mechanism.toposortedTree = toposort(tree(mechanism))
    mechanism
end

"""
$(SIGNATURES)

Return a dynamically equivalent `Mechanism`, but with a flat tree structure
with all bodies attached to the root body with a quaternion floating joint, and
with the 'tree edge' joints of the input `Mechanism` transformed into non-tree
edge joints (a constraint enforced using Lagrange multipliers in `dynamics!`).
"""
function maximal_coordinates(mechanism::Mechanism)
    T = eltype(mechanism)
    bodymap = Dict{RigidBody{T}, RigidBody{T}}()
    jointmap = Dict{Joint{T}, Joint{T}}()
    newfloatingjoints = Dict{RigidBody{T}, Joint{T}}()
    oldroot = root_body(mechanism)
    newroot = bodymap[oldroot] = deepcopy(oldroot)
    ret = Mechanism(newroot, gravity = mechanism.gravitationalAcceleration.v)

    # Copy bodies and attach them to the root with a floating joint.
    for oldbody in non_root_bodies(mechanism)
        newbody = bodymap[oldbody] = deepcopy(oldbody)
        frameBefore = default_frame(newroot)
        frameAfter = default_frame(newbody)
        floatingjoint = newfloatingjoints[newbody] = Joint(name(newbody), frameBefore, frameAfter, QuaternionFloating{T}())
        attach!(ret, newroot, floatingjoint, Transform3D(T, frameBefore), newbody, Transform3D(T, frameAfter))
    end

    # Copy input Mechanism's joints.
    function copy_edge(oldpredecessor::RigidBody, oldjoint::Joint, oldsuccessor::RigidBody)
        newjoint = jointmap[oldjoint] = deepcopy(oldjoint)
        jointToPredecessor = fixed_transform(mechanism, newjoint.frameBefore, default_frame(oldpredecessor))
        successorToJoint = fixed_transform(mechanism, default_frame(oldsuccessor), newjoint.frameAfter)
        attach!(ret, bodymap[oldpredecessor], newjoint, jointToPredecessor, bodymap[oldsuccessor], successorToJoint)
    end

    for vertex in non_root_vertices(mechanism)
        copy_edge(vertex_data(parent(vertex)), edge_to_parent_data(vertex), vertex_data(vertex))
    end

    for nonTreeEdge in mechanism.nonTreeEdges
        copy_edge(nonTreeEdge.predecessor, nonTreeEdge.successor, nonTreeEdge.joint)
    end

    ret, newfloatingjoints, bodymap, jointmap
end

"""
$(SIGNATURES)

Create a random `Mechanism` with the given joint types. Each new body is
attached to a parent selected using the `parentSelector` function.
"""
function rand_mechanism{T}(::Type{T}, parentSelector::Function, jointTypes...)
    parentBody = RigidBody{T}("world")
    mechanism = Mechanism(parentBody)
    for i = 1 : length(jointTypes)
        @assert jointTypes[i] <: JointType{T}
        joint = Joint("joint$i", rand(jointTypes[i]))
        jointToParentBody = rand(Transform3D{T}, joint.frameBefore, default_frame(parentBody))
        body = RigidBody(rand(SpatialInertia{T}, CartesianFrame3D("body$i")))
        bodyToJoint = Transform3D{T}(default_frame(body), joint.frameAfter)
        attach!(mechanism, parentBody, joint, jointToParentBody, body, bodyToJoint)
        parentBody = parentSelector(mechanism)
    end
    return mechanism
end

"""
$(SIGNATURES)

Create a random chain `Mechanism` with the given joint types.
"""
rand_chain_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, mechanism::Mechanism -> vertex_data(mechanism.toposortedTree[end]), jointTypes...)

"""
$(SIGNATURES)

Create a random tree `Mechanism` (without loops).
"""
rand_tree_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, mechanism::Mechanism -> rand(collect(bodies(mechanism))), jointTypes...)

"""
$(SIGNATURES)

Create a random tree `Mechanism` (without loops), with a quaternion floating
joint as the first joint (between the root body and the first non-root body).
"""
function rand_floating_tree_mechanism{T}(t::Type{T}, nonFloatingJointTypes...)
    parentSelector = (mechanism::Mechanism) -> begin
        only_root = length(bodies(mechanism)) == 1
        only_root ? root_body(mechanism) : rand(collect(non_root_bodies(mechanism)))
    end
    rand_mechanism(t, parentSelector, [QuaternionFloating{T}; nonFloatingJointTypes...]...)
end
