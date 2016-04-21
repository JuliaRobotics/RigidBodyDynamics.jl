type Mechanism{T}
    tree::Tree{RigidBody{T}, Joint}

    bodyFixedFrameDefinitions::Dict{RigidBody{T}, Vector{Transform3D{T}}}
    jointToJointTransforms::Dict{Joint, Transform3D{T}}

    function Mechanism(rootname::ASCIIString)
        root = RigidBody{T}(rootname)
        tree = Tree{RigidBody{T}, Joint}(root)
        bodyFixedFrameDefinitions = Dict{RigidBody{T}, Vector{Transform3D{T}}}(root => [])
        jointToJointTransforms = Dict{Joint, Transform3D{T}}()
        new(tree, bodyFixedFrameDefinitions, jointToJointTransforms)
    end
end
root(m::Mechanism) = m.tree.vertexData

function add_body_fixed_frame!{T}(m::Mechanism{T}, body::RigidBody{T}, transform::Transform3D{T})
    bodyVertex = findfirst(m.tree, body)
    target = isroot(bodyVertex) ? body.frame : bodyVertex.edgeToParentData.frameAfter
    fixedFrameDefinitions = m.bodyFixedFrameDefinitions[body]
    if transform.to != target
        index = findfirst(x -> x.from == transform.to && x.to == target, fixedFrameDefinitions)
        index == 0 && error("failed to add frame because transform doesn't connect to any known transforms")
        transform = fixedFrameDefinitions[index] * transform
    end
    push!(fixedFrameDefinitions, transform)
    return transform
end

function attach!{T}(m::Mechanism{T}, parentBody::RigidBody{T}, joint::Joint, jointToParent::Transform3D{T}, childBody::RigidBody{T}, childToJoint::Transform3D{T} = Transform3D{T}(childBody.frame))
    insert!(m.tree, childBody, joint, parentBody)
    m.jointToJointTransforms[joint] = add_body_fixed_frame!(m, parentBody, jointToParent)
    @assert childToJoint.from == childBody.frame
    @assert childToJoint.to == joint.frameAfter
    m.bodyFixedFrameDefinitions[childBody] = []
    if childToJoint.from != childToJoint.to
        push!(m.bodyFixedFrameDefinitions[childBody], childToJoint)
    end

    return m
end

joints{T}(m::Mechanism{T}) = keys(m.jointToJointTransforms)
bodies{T}(m::Mechanism{T}) = keys(m.bodyFixedFrameDefinitions)

num_positions{T}(m::Mechanism{T}) = reduce((val, joint) -> val + num_positions(joint), 0, joints(m))
num_velocities{T}(m::Mechanism{T}) = reduce((val, joint) -> val + num_velocities(joint), 0, joints(m))

immutable MechanismState{T}
    q::OrderedDict{Joint, Vector{T}}
    v::OrderedDict{Joint, Vector{T}}

    function MechanismState{M}(m::Mechanism{M})
        q = OrderedDict{Joint, Vector{T}}()
        v = OrderedDict{Joint, Vector{T}}()
        vertices = toposort(m.tree)
        for vertex in vertices
            if !isroot(vertex)
                joint = vertex.edgeToParentData
                q[joint] = Vector{T}(num_positions(joint))
                v[joint] = Vector{T}(num_velocities(joint))
            end
        end
        new(q, v)
    end
end

num_positions{T}(x::MechanismState{T}) = reduce((val, joint) -> val + num_positions(joint), 0, keys(x.q))
num_velocities{T}(x::MechanismState{T}) = reduce((val, joint) -> val + num_velocities(joint), 0, keys(x.v))

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
