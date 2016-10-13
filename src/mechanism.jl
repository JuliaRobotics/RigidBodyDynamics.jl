type Mechanism{T<:Real}
    toposortedTree::Vector{TreeVertex{RigidBody{T}, Joint{T}}}
    bodyFixedFrameDefinitions::Dict{RigidBody{T}, Set{Transform3D{T}}}
    bodyFixedFrameToBody::Dict{CartesianFrame3D, RigidBody{T}}
    jointToJointTransforms::Dict{Joint{T}, Transform3D{T}}
    gravitationalAcceleration::FreeVector3D{SVector{3, T}}
    qRanges::Dict{Joint{T}, UnitRange{Int64}} # TODO: remove
    vRanges::Dict{Joint{T}, UnitRange{Int64}} # TODO: remove

    function Mechanism(rootBody::RigidBody{T}; gravity::SVector{3, T} = SVector(zero(T), zero(T), T(-9.81)))
        tree = Tree{RigidBody{T}, Joint{T}}(rootBody)
        bodyFixedFrameDefinitions = Dict(rootBody => Set([Transform3D(T, rootBody.frame)]))
        bodyFixedFrameToBody = Dict(rootBody.frame => rootBody)
        jointToJointTransforms = Dict{Joint{T}, Transform3D{T}}()
        gravitationalAcceleration = FreeVector3D(rootBody.frame, gravity)
        qRanges = Dict{Joint{T}, UnitRange{Int64}}()
        vRanges = Dict{Joint{T}, UnitRange{Int64}}()
        new(toposort(tree), bodyFixedFrameDefinitions, bodyFixedFrameToBody, jointToJointTransforms, gravitationalAcceleration, qRanges, vRanges)
    end
end

Mechanism{T}(rootBody::RigidBody{T}; kwargs...) = Mechanism{T}(rootBody; kwargs...)
eltype{T}(::Mechanism{T}) = T
root_vertex(m::Mechanism) = m.toposortedTree[1]
non_root_vertices(m::Mechanism) = view(m.toposortedTree, 2 : length(m.toposortedTree))
tree(m::Mechanism) = m.toposortedTree[1]
root_body(m::Mechanism) = vertex_data(root_vertex(m))
root_frame(m::Mechanism) = root_body(m).frame
path(m::Mechanism, from::RigidBody, to::RigidBody) = path(findfirst(tree(m), from), findfirst(tree(m), to))
show(io::IO, m::Mechanism) = print(io, m.toposortedTree[1])
is_fixed_to_body{M}(m::Mechanism{M}, frame::CartesianFrame3D, body::RigidBody{M}) = body.frame == frame || any((t) -> t.from == frame, m.bodyFixedFrameDefinitions[body])
isinertial(m::Mechanism, frame::CartesianFrame3D) = is_fixed_to_body(m, frame, root_body(m))
isroot{T}(m::Mechanism{T}, b::RigidBody{T}) = b == root_body(m)

function default_frame{T}(m::Mechanism{T}, vertex::TreeVertex{RigidBody{T}, Joint{T}})
     # allows standardization on a frame to reduce number of transformations required
    isroot(vertex) ? vertex_data(vertex).frame : edge_to_parent_data(vertex).frameAfter
end

default_frame(m::Mechanism, body::RigidBody) = default_frame(m, findfirst(tree(m), body)) # TODO: potentially slow due to findfirst type instability

function find_body_fixed_frame_definition{T}(m::Mechanism{T}, body::RigidBody{T}, frame::CartesianFrame3D)::Transform3D{T}
    for transform in m.bodyFixedFrameDefinitions[body]
        transform.from == frame && return transform
    end
    error("$frame not found among body fixed frame definitions for $body")
end

function find_body_fixed_frame_definition(m::Mechanism, frame::CartesianFrame3D)
    find_body_fixed_frame_definition(m, m.bodyFixedFrameToBody[frame], frame)
end

function find_fixed_transform(m::Mechanism, from::CartesianFrame3D, to::CartesianFrame3D)
    body = m.bodyFixedFrameToBody[from]
    transform = find_body_fixed_frame_definition(m, body, from)
    if transform.to != to
        transform = inv(find_body_fixed_frame_definition(m, body, to)) * transform
    end
    transform
end

function add_body_fixed_frame!{T}(m::Mechanism{T}, body::RigidBody{T}, transform::Transform3D{T})
    definitions = get!(Set{Transform3D{T}}, m.bodyFixedFrameDefinitions, body)
    any((t) -> t.from == transform.from, definitions) && error("frame $(transform.from) was already defined")
    if transform.to != default_frame(m, body)
        transform = find_body_fixed_frame_definition(m, body, transform.to) * transform
    end
    push!(definitions, transform)
    m.bodyFixedFrameToBody[transform.from] = body
    return transform
end

function add_body_fixed_frame!{T}(m::Mechanism{T}, transform::Transform3D{T})
    body = m.bodyFixedFrameToBody[transform.to]
    body == nothing && error("unable to determine to what body $(transform.to) is attached")
    add_body_fixed_frame!(m, body, transform)
end

function canonicalize_frame_definitions!{T}(m::Mechanism{T}, vertex::TreeVertex{RigidBody{T}, Joint{T}})
    defaultFrame = default_frame(m, vertex)
    body = vertex_data(vertex)

    # set transform from joint to parent body's default frame
    if !isroot(vertex)
        parentDefaultFrame = default_frame(m, parent(vertex))
        joint = edge_to_parent_data(vertex)
        m.jointToJointTransforms[joint] = find_fixed_transform(m, joint.frameBefore, parentDefaultFrame)
    end

    # ensure that all body-fixed frame definitions map to default frame
    oldDefinitions = m.bodyFixedFrameDefinitions[body]
    newDefinitions = Set{Transform3D{T}}()
    for transform in oldDefinitions
        transform = transform.to == defaultFrame ? transform : find_fixed_transform(m, transform.to, default_frame(m, vertex)) * transform
        push!(newDefinitions, transform)
    end
    m.bodyFixedFrameDefinitions[body] = newDefinitions
    nothing
end

function recompute_ranges!(m::Mechanism)
    empty!(m.qRanges)
    empty!(m.vRanges)
    qStart, vStart = 1, 1
    for joint in joints(m)
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        m.qRanges[joint], m.vRanges[joint] = qStart : qEnd, vStart : vEnd
        qStart, vStart = qEnd + 1, vEnd + 1
    end
end

function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T},
        childBody::RigidBody{T}, childToJoint::Transform3D{T} = Transform3D{T}(childBody.frame, joint.frameAfter))
    vertex = insert!(tree(m), childBody, joint, parentBody)

    # define where joint is attached on parent body
    framecheck(jointToParent.from, joint.frameBefore)
    add_body_fixed_frame!(m, parentBody, jointToParent)

    # add identity transform from frame after joint to itself
    add_body_fixed_frame!(m, childBody, Transform3D{T}(joint.frameAfter, joint.frameAfter))

    # define where child is attached to joint
    framecheck(childToJoint.from, childBody.frame)
    if childToJoint.from != joint.frameAfter # we've already defined it
        add_body_fixed_frame!(m, childBody, childToJoint)
    end

    canonicalize_frame_definitions!(m, vertex)
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)
    m
end

# Essentially replaces the root body of childMechanism with parentBody (which belongs to m)
function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, childMechanism::Mechanism{T})
    # note: gravitational acceleration for childMechanism is ignored.

    # merge trees and set up frames for children of childMechanism's root
    parentVertex = findfirst(tree(m), parentBody)
    childRootVertex = root_vertex(childMechanism)
    childRootBody = vertex_data(childRootVertex)

    for child in children(childRootVertex)
        vertex = insert_subtree!(parentVertex, child)
    end

    # define where child root body is located w.r.t parent body
    add_body_fixed_frame!(m, parentBody, Transform3D{T}(childRootBody.frame, parentBody.frame)) # TODO: add optional function argument

    # add frames that were attached to childRootBody to parentBody
    for transform in childMechanism.bodyFixedFrameDefinitions[childRootBody]
        add_body_fixed_frame!(m, parentBody, transform)
    end

    canonicalize_frame_definitions!(m, parentVertex)
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)

    # merge frame info for vertices whose parents haven't changed
    childRootJoints = Joint[edge_to_parent_data(child) for child in children(childRootVertex)]
    merge!(m.bodyFixedFrameDefinitions, filter((k, v) -> k != childRootBody, childMechanism.bodyFixedFrameDefinitions))
    merge!(m.bodyFixedFrameToBody, filter((k, v) -> v != childRootBody, childMechanism.bodyFixedFrameToBody))
    merge!(m.jointToJointTransforms, filter((k, v) -> k ∉ childRootJoints, childMechanism.jointToJointTransforms))

    m
end

function submechanism{T}(m::Mechanism{T}, submechanismRoot::RigidBody{T})
    # Create mechanism and set up tree
    ret = Mechanism{T}(submechanismRoot; gravity = m.gravitationalAcceleration.v)
    for child in children(findfirst(tree(m), submechanismRoot))
        insert_subtree!(root_vertex(ret), child)
    end
    ret.toposortedTree = toposort(tree(ret))
    recompute_ranges!(ret)

    # copy frame information over
    merge!(ret.bodyFixedFrameDefinitions, filter((k, v) -> k ∈ bodies(ret), m.bodyFixedFrameDefinitions))
    merge!(ret.bodyFixedFrameToBody, filter((k, v) -> v ∈ bodies(ret), m.bodyFixedFrameToBody))
    merge!(ret.jointToJointTransforms, filter((k, v) -> k ∈ joints(ret), m.jointToJointTransforms))

    # update frame definitions associated with root
    formerJointToRootBody = inv(find_body_fixed_frame_definition(ret, submechanismRoot, submechanismRoot.frame))
    newFrameDefinitions = map(t -> formerJointToRootBody * t, ret.bodyFixedFrameDefinitions[submechanismRoot])
    push!(newFrameDefinitions, formerJointToRootBody)
    ret.bodyFixedFrameDefinitions[submechanismRoot] = newFrameDefinitions

    for child in children(findfirst(tree(m), submechanismRoot))
        joint = edge_to_parent_data(child)
        ret.jointToJointTransforms[joint] = formerJointToRootBody * ret.jointToJointTransforms[joint]
    end

    ret
end

#=
Reroots the subtree of the root body to which newSubtreeRootBody belongs so that newSubtreeRootBody is attached to
the root body with the given joint.
=#
function reroot_subtree!{T}(mechanism::Mechanism{T}, newSubtreeRootBody::RigidBody{T}, joint::Joint{T},
        jointToWorld::Transform3D{T}, newSubTreeRootBodyToJoint::Transform3D{T})
    # TODO: add option to prune frames related to old joints

    newSubtreeRootBody == root_body(mechanism) && error("new subtree root must be part of a subtree of mechanism root")
    newSubtreeRoot = findfirst(tree(mechanism), newSubtreeRootBody)

    # modify tree
    oldSubtreeRoot = newSubtreeRoot
    while !isroot(parent(oldSubtreeRoot))
        oldSubtreeRoot = parent(oldSubtreeRoot)
    end
    oldRootJoint = edge_to_parent_data(oldSubtreeRoot)
    delete!(mechanism.jointToJointTransforms, oldRootJoint)
    detach!(oldSubtreeRoot)
    flippedJoints = Pair{Joint{T}, Joint{T}}[]
    flipDirectionFunction = joint -> begin
        flipped = Joint(joint.name * "Flipped", joint.jointType) # TODO
        push!(flippedJoints, joint => flipped)
        flipped
    end
    subtreeRerooted = reroot(newSubtreeRoot, flipDirectionFunction)
    insert!(root_vertex(mechanism), subtreeRerooted, joint)

    # define frames related to new joint
    # TODO: replace with call to attach!
    add_body_fixed_frame!(mechanism, root_body(mechanism), jointToWorld)
#     add_body_fixed_frame!(mechanism, newSubtreeRootBody, Transform3D{T}(joint.frameAfter, joint.frameAfter)) TODO
    add_body_fixed_frame!(mechanism, newSubtreeRootBody, inv(newSubTreeRootBodyToJoint))

    # define identities between new frames and old frames
    for pair in flippedJoints
        oldJoint, newJoint = first(pair), last(pair)
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameBefore, oldJoint.frameAfter))
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameAfter, oldJoint.frameBefore))
    end

    # canonicalize frame definitions
    for vertex in toposort(tree(mechanism))
        canonicalize_frame_definitions!(mechanism, vertex)
    end

    mechanism.toposortedTree = toposort(tree(mechanism))
    recompute_ranges!(mechanism)
    mechanism
end

function change_joint_type!(m::Mechanism, joint::Joint, newType::JointType)
    # TODO: remove ranges from mechanism so that this function isn't necessary
    joint.jointType = newType
    recompute_ranges!(m::Mechanism)
    m
end

function remove_fixed_joints!(m::Mechanism)
    T = eltype(m)
    for vertex in copy(m.toposortedTree)
        if !isroot(vertex)
            parentVertex = parent(vertex)
            body = vertex_data(vertex)
            joint = edge_to_parent_data(vertex)
            if isa(joint.jointType, Fixed)
                jointTransform = Transform3D{T}(joint.frameAfter, joint.frameBefore)
                afterJointToParentJoint = m.jointToJointTransforms[joint] * jointTransform

                # add inertia to parent body
                parentBody = vertex_data(parent(vertex))
                if has_defined_inertia(parentBody)
                    inertia = spatial_inertia(body)
                    inertiaFrameToFrameAfterJoint = find_body_fixed_frame_definition(m, body, inertia.frame)

                    parentInertia = spatial_inertia(parentBody)
                    parentInertiaFrameToParentJoint = find_body_fixed_frame_definition(m, parentBody, parentInertia.frame)

                    inertiaToParentInertia = inv(parentInertiaFrameToParentJoint) * afterJointToParentJoint * inertiaFrameToFrameAfterJoint
                    parentBody.inertia = parentInertia + transform(inertia, inertiaToParentInertia)
                end

                # update children's joint to parent transforms
                for child in copy(children(vertex))
                    childJoint = edge_to_parent_data(child)
                    m.jointToJointTransforms[childJoint] = afterJointToParentJoint * m.jointToJointTransforms[childJoint]
                end
                delete!(m.jointToJointTransforms, joint)

                # add body fixed frames to parent body
                for transform in m.bodyFixedFrameDefinitions[body]
                    transform = afterJointToParentJoint * transform
                    add_body_fixed_frame!(m, parentBody, transform)
                end
                delete!(m.bodyFixedFrameDefinitions, body)
                delete!(m.bodyFixedFrameToBody, body)

                # merge vertex into parent
                merge_into_parent!(vertex)
            end
        end
    end
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)
    m
end

joints(m::Mechanism) = [edge_to_parent_data(vertex) for vertex in non_root_vertices(m)] # TODO: make less expensive
bodies{T}(m::Mechanism{T}) = [vertex_data(vertex)::RigidBody{T} for vertex in m.toposortedTree] # TODO: make less expensive
non_root_bodies{T}(m::Mechanism{T}) = [vertex_data(vertex)::RigidBody{T} for vertex in non_root_vertices(m)] # TODO: make less expensive

num_positions(m::Mechanism) = num_positions(joints(m))
num_velocities(m::Mechanism) = num_velocities(joints(m))

function rand_mechanism{T}(::Type{T}, parentSelector::Function, jointTypes...)
    parentBody = RigidBody{T}("world")
    m = Mechanism(parentBody)
    for i = 1 : length(jointTypes)
        @assert jointTypes[i] <: JointType{T}
        joint = Joint("joint$i", rand(jointTypes[i]))
        jointToParentBody = rand(Transform3D{T}, joint.frameBefore, parentBody.frame)
        body = RigidBody(rand(SpatialInertia{T}, CartesianFrame3D("body$i")))
        bodyToJoint = Transform3D{Float64}(body.frame, joint.frameAfter) #rand(Transform3D{Float64}, body.frame, joint.frameAfter)
        attach!(m, parentBody, joint, jointToParentBody, body, bodyToJoint)
        parentBody = parentSelector(m)
    end
    return m
end

rand_chain_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, m::Mechanism -> vertex_data(m.toposortedTree[end]), jointTypes...)
rand_tree_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, m::Mechanism -> rand(collect(bodies(m))), jointTypes...)

function gravitational_spatial_acceleration{M}(m::Mechanism{M})
    frame = m.gravitationalAcceleration.frame
    SpatialAcceleration(frame, frame, frame, zeros(SVector{3, M}), m.gravitationalAcceleration.v)
end
