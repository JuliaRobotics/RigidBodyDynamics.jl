type Mechanism{T<:Real}
    toposortedTree::Vector{TreeVertex{RigidBody{T}, Joint{T}}} # TODO: consider replacing with just the root vertex after creating iterator
    bodyFixedFrameDefinitions::Dict{RigidBody{T}, Set{Transform3D{T}}}
    bodyFixedFrameToBody::Dict{CartesianFrame3D, RigidBody{T}}
    jointToJointTransforms::Dict{Joint{T}, Transform3D{T}}
    gravitationalAcceleration::FreeVector3D{SVector{3, T}} # TODO: consider removing
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
        new(toposort(tree), bodyFixedFrameDefinitions, bodyFixedFrameToBody,
            jointToJointTransforms, gravitationalAcceleration, qRanges, vRanges)
    end

    function Mechanism(m::Mechanism{T})
        new(toposort(copy(tree(m))), Dict(k => copy(v) for (k, v) in m.bodyFixedFrameDefinitions), copy(m.bodyFixedFrameToBody),
            copy(m.jointToJointTransforms), copy(m.gravitationalAcceleration), copy(m.qRanges), copy(m.vRanges))
    end
end

Mechanism{T}(rootBody::RigidBody{T}; kwargs...) = Mechanism{T}(rootBody; kwargs...)
Mechanism{T}(m::Mechanism{T}) = Mechanism{T}(m)
copy(m::Mechanism) = Mechanism(m)
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
joints(m::Mechanism) = (edge_to_parent_data(vertex) for vertex in non_root_vertices(m))
bodies{T}(m::Mechanism{T}) = (vertex_data(vertex) for vertex in m.toposortedTree)
non_root_bodies{T}(m::Mechanism{T}) = (vertex_data(vertex) for vertex in non_root_vertices(m))
num_positions(m::Mechanism) = num_positions(joints(m))
num_velocities(m::Mechanism) = num_velocities(joints(m))
num_bodies(m::Mechanism) = length(m.toposortedTree)

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
    # note: overwrites any existing frame definition
    definitions = get!(Set{Transform3D{T}}, m.bodyFixedFrameDefinitions, body)
    if transform.to != default_frame(m, body)
        transform = find_body_fixed_frame_definition(m, body, transform.to) * transform
    end
    filter!(t -> t.from != transform.from, definitions)
    push!(definitions, transform)
    m.bodyFixedFrameToBody[transform.from] = body
    transform
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
    # and that there is a transform from the default frame to itself # TODO: reconsider requiring this
    oldDefinitions = m.bodyFixedFrameDefinitions[body]
    newDefinitions = Set{Transform3D{T}}()
    identityFound = false
    for transform in oldDefinitions
        transform = transform.to == defaultFrame ? transform : find_fixed_transform(m, transform.to, default_frame(m, vertex)) * transform
        identityFound = identityFound || transform.from == defaultFrame
        push!(newDefinitions, transform)
    end
    if !identityFound
        push!(newDefinitions, Transform3D{T}(defaultFrame, defaultFrame))
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
    add_body_fixed_frame!(m, childBody, childToJoint)

    canonicalize_frame_definitions!(m, vertex)
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)
    m
end

# Essentially replaces the root body of childMechanism with parentBody (which belongs to m)
function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, childMechanism::Mechanism{T})
    # note: gravitational acceleration for childMechanism is ignored.

    parentVertex = findfirst(tree(m), parentBody)
    childRootVertex = root_vertex(childMechanism)
    childRootBody = vertex_data(childRootVertex)

    # define where child root body is located w.r.t parent body
    # and add frames that were attached to childRootBody to parentBody
    add_body_fixed_frame!(m, parentBody, Transform3D{T}(childRootBody.frame, parentBody.frame)) # TODO: add optional function argument
    for transform in childMechanism.bodyFixedFrameDefinitions[childRootBody]
        add_body_fixed_frame!(m, parentBody, transform)
    end
    canonicalize_frame_definitions!(m, parentVertex)

    # merge in frame info for vertices whose parents will remain the same
    merge!(m.bodyFixedFrameDefinitions, filter((k, v) -> k != childRootBody, childMechanism.bodyFixedFrameDefinitions))
    merge!(m.bodyFixedFrameToBody, filter((k, v) -> v != childRootBody, childMechanism.bodyFixedFrameToBody))
    merge!(m.jointToJointTransforms, childMechanism.jointToJointTransforms)

    # merge trees
    for child in children(childRootVertex)
        vertex = insert_subtree!(parentVertex, child)
        canonicalize_frame_definitions!(m, vertex)
    end

    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)

    m
end

function submechanism{T}(m::Mechanism{T}, submechanismRootBody::RigidBody{T})
    # Create mechanism and set up tree
    ret = Mechanism{T}(submechanismRootBody; gravity = m.gravitationalAcceleration.v)
    for child in children(findfirst(tree(m), submechanismRootBody))
        insert_subtree!(root_vertex(ret), child)
    end
    ret.toposortedTree = toposort(tree(ret))
    recompute_ranges!(ret)

    # copy frame information over
    merge!(ret.bodyFixedFrameDefinitions, filter((k, v) -> k ∈ bodies(ret), m.bodyFixedFrameDefinitions))
    merge!(ret.bodyFixedFrameToBody, filter((k, v) -> v ∈ bodies(ret), m.bodyFixedFrameToBody))
    merge!(ret.jointToJointTransforms, filter((k, v) -> k ∈ joints(ret), m.jointToJointTransforms))

    canonicalize_frame_definitions!(ret, root_vertex(ret))

    ret
end

#=
Detaches the subtree rooted at oldSubtreeRootBody, reroots it so that newSubtreeRootBody is the new root, and then attaches
newSubtreeRootBody to parentBody using the specified joint.
=#
function reattach!{T}(mechanism::Mechanism{T}, oldSubtreeRootBody::RigidBody{T},
        parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T},
        newSubtreeRootBody::RigidBody{T}, newSubTreeRootBodyToJoint::Transform3D{T} = Transform3D{T}(newSubtreeRootBody.frame, joint.frameAfter))
    # TODO: add option to prune frames related to old joints

    newSubtreeRoot = findfirst(tree(mechanism), newSubtreeRootBody)
    oldSubtreeRoot = findfirst(tree(mechanism), oldSubtreeRootBody)
    parentVertex = findfirst(tree(mechanism), parentBody)
    @assert newSubtreeRoot ∈ toposort(oldSubtreeRoot)
    @assert parentVertex ∉ toposort(oldSubtreeRoot)

    # detach oldSubtreeRoot (but keep body-fixed frame information)
    oldRootJoint = edge_to_parent_data(oldSubtreeRoot)
    delete!(mechanism.jointToJointTransforms, oldRootJoint)
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
    add_body_fixed_frame!(mechanism, parentBody, jointToParent)
    add_body_fixed_frame!(mechanism, newSubtreeRootBody, inv(newSubTreeRootBodyToJoint))

    # define identities between new frames and old frames and recanonicalize frame definitions
    for (oldJoint, newJoint) in flippedJoints
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameBefore, oldJoint.frameAfter))
        add_body_fixed_frame!(mechanism, Transform3D{T}(newJoint.frameAfter, oldJoint.frameBefore))
    end
    for vertex in mechanism.toposortedTree
        canonicalize_frame_definitions!(mechanism, vertex)
    end

    recompute_ranges!(mechanism)
    flippedJoints
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
            body = vertex_data(vertex)
            joint = edge_to_parent_data(vertex)
            parentBody = vertex_data(parent(vertex))
            if isa(joint.jointType, Fixed)
                # add identity joint transform as a body-fixed frame definition
                jointTransform = Transform3D{T}(joint.frameAfter, joint.frameBefore)
                add_body_fixed_frame!(m, parentBody, jointTransform)
                delete!(m.jointToJointTransforms, joint)

                # migrate body fixed frames to parent body
                for transform in m.bodyFixedFrameDefinitions[body]
                    add_body_fixed_frame!(m, parentBody, transform)
                end
                delete!(m.bodyFixedFrameDefinitions, body) # TODO: remove_body_fixed_frame!
                delete!(m.bodyFixedFrameToBody, body)

                # add inertia to parent body
                if has_defined_inertia(parentBody)
                    inertia = spatial_inertia(body)
                    parentInertia = spatial_inertia(parentBody)
                    toParent = find_fixed_transform(m, inertia.frame, parentInertia.frame)
                    parentBody.inertia = parentInertia + transform(inertia, toParent)
                end

                # merge vertex into parent
                merge_into_parent!(vertex)
            end
        end
    end
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)
    m
end

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
function rand_floating_tree_mechanism{T}(t::Type{T}, nonFloatingJointTypes...)
    parentSelector = (m::Mechanism) -> begin
        only_root = length(bodies(m)) == 1
        only_root ? root_body(m) : rand(collect(non_root_bodies(m)))
    end
    rand_mechanism(t, parentSelector, [QuaternionFloating{T}; nonFloatingJointTypes...]...)
end

function gravitational_spatial_acceleration{M}(m::Mechanism{M})
    frame = m.gravitationalAcceleration.frame
    SpatialAcceleration(frame, frame, frame, zeros(SVector{3, M}), m.gravitationalAcceleration.v)
end
