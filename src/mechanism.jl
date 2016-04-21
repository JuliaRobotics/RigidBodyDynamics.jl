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
root{T}(m::Mechanism{T}) = m.tree.vertexData

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
    if childToJoint.from != childToJoint.to
        m.bodyFixedFrameDefinitions[childBody] = [childToJoint]
    else
        m.bodyFixedFrameDefinitions[childBody] = []
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
    for kv in x.q
        zero_configuration!(first(kv), last(kv))
    end
end
configuration_vector{T}(x::MechanismState{T}) = vcat(values(x.q)...)
velocity_vector{T}(x::MechanismState{T}) = vcat(values(x.v)...)

function FrameCache{M, X}(m::Mechanism{M}, x::MechanismState{X})
    cache = FrameCache{promote_type(M, X)}(root(m).frame)

    # first march down the tree
    vertices = toposort(m.tree)
    for vertex in vertices
        if !isroot(vertex)
            joint = vertex.edgeToParentData
            add_frame!(cache, m.jointToJointTransforms[joint])
            qJoint = x.q[joint]
            add_frame!(cache, () -> joint_transform(joint, qJoint))
        end
    end

    # then add body fixed frames
    for transforms in values(m.bodyFixedFrameDefinitions)
        for transform in transforms
            add_frame!(cache, transform)
        end
    end

    setdirty!(cache)
    return cache
end

function subtree_mass{M}(m::Mechanism{M}, base::Tree{RigidBody{M}, Joint})
    if isroot(base)
        result = 0
    else
        result = base.vertexData.inertia.mass
    end
    for child in base.children
        result += subtree_mass(m, child)
    end
    return result
end
mass{M}(m::Mechanism{M}) = subtree_mass(m, m.tree)

function center_of_mass{C}(itr, frame::CartesianFrame3D, cache::FrameCache{C})
    com = Point3D(frame, zero(Vec{3, C}))
    mass = zero(C)
    for body in itr
        if !isroot(body)
            inertia = body.inertia
            com += inertia.mass * transform(cache, Point3D(inertia.frame, inertia.centerOfMass), frame)
            mass += inertia.mass
        end
    end
    com /= mass
    return com
end

center_of_mass{M, C}(m::Mechanism{M}, cache::FrameCache{C}) = center_of_mass(bodies(m), root(m).frame, cache)
