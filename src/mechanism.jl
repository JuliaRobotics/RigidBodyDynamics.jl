type Mechanism{T<:Real}
    toposortedTree::Vector{TreeVertex{RigidBody{T}, Joint}}
    bodyFixedFrameDefinitions::Dict{RigidBody{T}, Set{Transform3D{T}}}
    bodyFixedFrameToBody::Dict{CartesianFrame3D, RigidBody{T}}
    jointToJointTransforms::Dict{Joint, Transform3D{T}}

    function Mechanism(rootname::ASCIIString)
        rootBody = RigidBody{T}(rootname)
        tree = Tree{RigidBody{T}, Joint}(rootBody)
        bodyFixedFrameDefinitions = Dict{RigidBody{T}, Set{Transform3D{T}}}(rootBody => Set([Transform3D(T, rootBody.frame)]))
        bodyFixedFrameToBody = Dict{CartesianFrame3D, RigidBody{T}}(rootBody.frame => rootBody)
        jointToJointTransforms = Dict{Joint, Transform3D{T}}()
        new(toposort(tree), bodyFixedFrameDefinitions, bodyFixedFrameToBody, jointToJointTransforms)
    end
end
root_vertex(m::Mechanism) = m.toposortedTree[1]
tree(m::Mechanism) = m.toposortedTree[1]
root_body(m::Mechanism) = root_vertex(m).vertexData
root_frame(m::Mechanism) = root_body(m).frame
path(m::Mechanism, from::RigidBody, to::RigidBody) = path(findfirst(tree(m), from), findfirst(tree(m), to))

function add_body_fixed_frame!{T}(m::Mechanism{T}, body::RigidBody{T}, transform::Transform3D{T})
    fixedFrameDefinitions = m.bodyFixedFrameDefinitions[body]
    any((t) -> t.from == transform.from, fixedFrameDefinitions) && error("frame was already defined")
    bodyVertex = findfirst(tree(m), body)
    default_frame = isroot(bodyVertex) ? body.frame : bodyVertex.edgeToParentData.frameAfter
    if transform.to != default_frame
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

function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T}, childBody::RigidBody{T}, childToJoint::Transform3D{T} = Transform3D{T}(childBody.frame))
    insert!(tree(m), childBody, joint, parentBody)
    m.jointToJointTransforms[joint] = add_body_fixed_frame!(m, parentBody, jointToParent)
    @assert childToJoint.from == childBody.frame
    @assert childToJoint.to == joint.frameAfter
    m.bodyFixedFrameDefinitions[childBody] = Set([Transform3D(T, joint.frameAfter)])
    m.bodyFixedFrameToBody[joint.frameAfter] = childBody
    if childToJoint.from != childToJoint.to
        push!(m.bodyFixedFrameDefinitions[childBody], childToJoint)
        m.bodyFixedFrameToBody[childToJoint.from] = childBody
    end
    m.toposortedTree = toposort(tree(m))
    return m
end

joints{T}(m::Mechanism{T}) = keys(m.jointToJointTransforms)
bodies{T}(m::Mechanism{T}) = keys(m.bodyFixedFrameDefinitions)
default_frame(m::Mechanism, body::RigidBody) = first(bodyFixedFrameDefinitions[body]).to # allows standardization on a frame to reduce number of transformations required

num_positions{T}(m::Mechanism{T}) = num_positions(joints(m))
num_velocities{T}(m::Mechanism{T}) = num_velocities(joints(m))

immutable MechanismState{T<:Real}
    q::OrderedDict{Joint, Vector{T}}
    v::OrderedDict{Joint, Vector{T}}

    function MechanismState{M}(m::Mechanism{M})
        q = OrderedDict{Joint, Vector{T}}()
        v = OrderedDict{Joint, Vector{T}}()
        for vertex in m.toposortedTree
            if !isroot(vertex)
                joint = vertex.edgeToParentData
                q[joint] = Vector{T}(num_positions(joint))
                v[joint] = Vector{T}(num_velocities(joint))
            end
        end
        new(q, v)
    end
end

num_positions{T}(x::MechanismState{T}) = num_positions(keys(x.q))
num_velocities{T}(x::MechanismState{T}) = num_velocities(keys(x.v))

function zero_configuration!{T}(x::MechanismState{T})
    for joint in keys(x.q) x.q[joint] = zero_configuration(joint, T) end
end
function zero_velocity!(x::MechanismState)
    for joint in keys(x.v) x.v[joint] = zero(x.v[joint]) end
end
zero!(x::MechanismState) = begin zero_configuration!(x); zero_velocity!(x) end

function rand_configuration!{T}(x::MechanismState{T})
    for joint in keys(x.q) x.q[joint] = rand_configuration(joint, T) end
end
function rand_velocity!(x::MechanismState)
    for joint in keys(x.v) x.v[joint] = rand!(x.v[joint]) end
end
rand!(x::MechanismState) = begin rand_configuration!(x); rand_velocity!(x) end

configuration_vector{T}(x::MechanismState{T}) = vcat(values(x.q)...)
velocity_vector{T}(x::MechanismState{T}) = vcat(values(x.v)...)
