type Mechanism{T<:Real}
    toposortedTree::Vector{TreeVertex{RigidBody{T}, Joint}}
    bodyFixedFrameDefinitions::Dict{RigidBody{T}, Set{Transform3D{T}}}
    bodyFixedFrameToBody::Dict{CartesianFrame3D, RigidBody{T}}
    jointToJointTransforms::Dict{Joint, Transform3D{T}}
    gravitationalAcceleration::FreeVector3D{SVector{3, T}}
    qRanges::Dict{Joint, UnitRange{Int64}}
    vRanges::Dict{Joint, UnitRange{Int64}}

    function Mechanism(rootBody::RigidBody{T}; gravity::SVector{3, T} = SVector(zero(T), zero(T), T(-9.81)))
        tree = Tree{RigidBody{T}, Joint}(rootBody)
        bodyFixedFrameDefinitions = Dict(rootBody => Set([Transform3D(T, rootBody.frame)]))
        bodyFixedFrameToBody = Dict(rootBody.frame => rootBody)
        jointToJointTransforms = Dict{Joint, Transform3D{T}}()
        gravitationalAcceleration = FreeVector3D(rootBody.frame, gravity)
        qRanges = Dict{Joint, UnitRange{Int64}}()
        vRanges = Dict{Joint, UnitRange{Int64}}()
        new(toposort(tree), bodyFixedFrameDefinitions, bodyFixedFrameToBody, jointToJointTransforms, gravitationalAcceleration, qRanges, vRanges)
    end
end

Mechanism{T}(rootBody::RigidBody{T}; kwargs...) = Mechanism{T}(rootBody; kwargs...)
root_vertex(m::Mechanism) = m.toposortedTree[1]
non_root_vertices(m::Mechanism) = view(m.toposortedTree, 2 : length(m.toposortedTree))
tree(m::Mechanism) = m.toposortedTree[1]
root_body(m::Mechanism) = root_vertex(m).vertexData
root_frame(m::Mechanism) = root_body(m).frame
path(m::Mechanism, from::RigidBody, to::RigidBody) = path(findfirst(tree(m), from), findfirst(tree(m), to))
show(io::IO, m::Mechanism) = print(io, m.toposortedTree[1])
is_fixed_to_body{M}(m::Mechanism{M}, frame::CartesianFrame3D, body::RigidBody{M}) = body.frame == frame || any((t) -> t.from == frame, bodyFixedFrameDefinitions[body])
isinertial(m::Mechanism, frame::CartesianFrame3D) = is_fixed_to_body(m, frame, root_body(m))
isroot{T}(m::Mechanism{T}, b::RigidBody{T}) = b == root_body(m)

function add_body_fixed_frame!{T}(m::Mechanism{T}, body::RigidBody{T}, transform::Transform3D{T})
    fixedFrameDefinitions = m.bodyFixedFrameDefinitions[body]
    any((t) -> t.from == transform.from, fixedFrameDefinitions) && error("frame $(transform.from) was already defined")
    bodyVertex = findfirst(tree(m), body)
    defaultFrame = isroot(bodyVertex) ? body.frame : bodyVertex.edgeToParentData.frameAfter
    if transform.to != defaultFrame
        found = false
        for t in fixedFrameDefinitions
            if t.from == transform.to
                found = true
                transform = t * transform
                break
            end
        end
        !found && error("failed to add frame because transform doesn't connect to any known transforms")
    end
    push!(fixedFrameDefinitions, transform)
    m.bodyFixedFrameToBody[transform.from] = body
    return transform
end

function recompute_ranges!(m::Mechanism)
    qStart, vStart = 1, 1
    for joint in joints(m)
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        m.qRanges[joint], m.vRanges[joint] = qStart : qEnd, vStart : vEnd
        qStart, vStart = qEnd + 1, vEnd + 1
    end
end

function set_up_frames!{T}(m::Mechanism{T}, vertex::TreeVertex{RigidBody{T}, Joint},
        jointToParent::Transform3D{T}, bodyToJoint::Transform3D{T})
    joint = vertex.edgeToParentData
    body = vertex.vertexData
    parentBody = vertex.parent.vertexData

    # add transform from frame before joint to parent body's default frame
    m.jointToJointTransforms[joint] = add_body_fixed_frame!(m, parentBody, jointToParent)

    # add transform from body to frame after joint to body fixed frame definitions
    framecheck(bodyToJoint.from, body.frame)
    framecheck(bodyToJoint.to, joint.frameAfter)
    m.bodyFixedFrameToBody[joint.frameAfter] = body
    m.bodyFixedFrameDefinitions[body] = Set([Transform3D(T, joint.frameAfter)])
    if bodyToJoint.from != bodyToJoint.to
        push!(m.bodyFixedFrameDefinitions[body], bodyToJoint)
        m.bodyFixedFrameToBody[bodyToJoint.from] = body
    end
end

function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T},
        childBody::RigidBody{T}, childToJoint::Transform3D{T} = Transform3D{T}(childBody.frame, joint.frameAfter))
    vertex = insert!(tree(m), childBody, joint, parentBody)
    set_up_frames!(m, vertex, jointToParent, childToJoint)
    push!(m.toposortedTree, vertex)
    recompute_ranges!(m)
    m
end

# Essentially replaces the root body of childMechanism with parentBody (which belongs to m)
function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, childMechanism::Mechanism{T})
    # note: gravitational acceleration for childMechanism is ignored.

    # merge trees and set up frames for children of childMechanism's root
    parentVertex = findfirst(tree(m), parentBody)
    childRootVertex = root_vertex(childMechanism)
    childRootBody = childRootVertex.vertexData
    childRootBodyToParentBody = Transform3D{T}(childRootBody.frame, parentBody.frame) # identity
    for child in childRootVertex.children
        vertex = insert_subtree!(parentVertex, child)
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        jointToParent = childRootBodyToParentBody * childMechanism.jointToJointTransforms[joint]
        bodyToJointSet = filter(transform -> transform.from == body.frame, childMechanism.bodyFixedFrameDefinitions[body])
        @assert length(bodyToJointSet) == 1
        bodyToJoint = first(bodyToJointSet)
        set_up_frames!(m, vertex, jointToParent, bodyToJoint)
    end
    m.toposortedTree = toposort(tree(m))
    recompute_ranges!(m)

    # add frames that were attached to childRootBody to parentBody
    for transform in childMechanism.bodyFixedFrameDefinitions[childRootBody]
        if isempty(filter(t -> t.from == transform.from, m.bodyFixedFrameDefinitions[parentBody]))
            add_body_fixed_frame!(m, parentBody, childRootBodyToParentBody * transform)
        end
    end

    # merge frame info for vertices whose parents haven't changed
    childRootJoints = Joint[child.edgeToParentData for child in childRootVertex.children]
    merge!(m.bodyFixedFrameDefinitions, filter((k, v) -> k != childRootBody, childMechanism.bodyFixedFrameDefinitions))
    merge!(m.bodyFixedFrameToBody, filter((k, v) -> v != childRootBody, childMechanism.bodyFixedFrameToBody))
    merge!(m.jointToJointTransforms, filter((k, v) -> k ∉ childRootJoints, childMechanism.jointToJointTransforms))

    m
end

joints(m::Mechanism) = [vertex.edgeToParentData::Joint for vertex in non_root_vertices(m)] # TODO: make less expensive
bodies{T}(m::Mechanism{T}) = [vertex.vertexData::RigidBody{T} for vertex in m.toposortedTree] # TODO: make less expensive
non_root_bodies{T}(m::Mechanism{T}) = [vertex.vertexData::RigidBody{T} for vertex in non_root_vertices(m)] # TODO: make less expensive
default_frame(m::Mechanism, body::RigidBody) = first(m.bodyFixedFrameDefinitions[body]).to # allows standardization on a frame to reduce number of transformations required

num_positions(m::Mechanism) = num_positions(joints(m))
num_velocities(m::Mechanism) = num_velocities(joints(m))

function rand_mechanism{T}(::Type{T}, parentSelector::Function, jointTypes...)
    parentBody = RigidBody{T}("world")
    m = Mechanism(parentBody)
    for i = 1 : length(jointTypes)
        @assert jointTypes[i] <: JointType
        joint = Joint("joint$i", rand(jointTypes[i]))
        jointToParentBody = rand(Transform3D{T}, joint.frameBefore, parentBody.frame)
        body = RigidBody(rand(SpatialInertia{T}, CartesianFrame3D("body$i")))
        bodyToJoint = Transform3D{Float64}(body.frame, joint.frameAfter) #rand(Transform3D{Float64}, body.frame, joint.frameAfter)
        attach!(m, parentBody, joint, jointToParentBody, body, bodyToJoint)
        parentBody = parentSelector(m)
    end
    return m
end

rand_chain_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, m::Mechanism -> m.toposortedTree[end].vertexData, jointTypes...)
rand_tree_mechanism{T}(t::Type{T}, jointTypes...) = rand_mechanism(t, m::Mechanism -> rand(collect(bodies(m))), jointTypes...)

function gravitational_spatial_acceleration{M}(m::Mechanism{M})
    frame = m.gravitationalAcceleration.frame
    SpatialAcceleration(frame, frame, frame, zeros(SVector{3, M}), m.gravitationalAcceleration.v)
end
