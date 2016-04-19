immutable SpatialInertia{T}
    frame::CartesianFrame3D
    moment::Mat{3, 3, T}
    centerOfMass::Vec{3, T}
    mass::T
end

function transform{T}(inertia::SpatialInertia{T}, t::Transform3D{T})
    @assert t.from == inertia.frame

    function vector_to_skew_symmetric_squared(a::Vec{3, T})
        aSq = a .* a
        b11 = -aSq[2] - aSq[3]
        b12 = a[1] * a[2]
        b13 = a[1] * a[3]
        b22 = -aSq[1] - aSq[3]
        b23 = a[2] * a[3]
        b33 = -aSq[1] - aSq[2]
        return @fsa([b11 b12 b13; b12 b22 b23; b13 b23 b33])
    end

    J = inertia.moment
    m = inertia.mass
    c = inertia.centerOfMass

    R = Mat(rotationmatrix(t.rot))
    p = t.trans

    cnew = R * (c * m)
    Jnew = vector_to_skew_symmetric_squared(cnew)
    cnew += m * p
    Jnew -= vector_to_skew_symmetric_squared(cnew)
    Jnew /= m
    Jnew += R * J * R'
    cnew /= m

    return SpatialInertia(t.to, Jnew, cnew, m)
end

immutable RigidBody
    frame::CartesianFrame3D
    inertia::Nullable{SpatialInertia{Float64}}

    # world body
    RigidBody(name::ASCIIString) = new(CartesianFrame3D(name), null)

    # other bodies
    RigidBody(inertia::SpatialInertia{Float64}) = new(inertia.frame, inertia)
end
name(b::RigidBody) = b.frame.name
isroot(b::RigidBody) = isnull(b.inertia)

immutable SpatialMotionMatrix{N, T}
    body::RigidBody
    base::RigidBody
    frame::CartesianFrame3D
    angular::Mat{3, N, T}
    linear::Mat{3, N, T}
end
typealias SpatialMotionVector{T} SpatialMotionMatrix{1, T}

function transform{N, T}(m::SpatialMotionMatrix{N, T}, t::Transform3D{T})
    @assert m.frame == t.from
    angular = rotate(m.angular, t.rot)
    linear = rotate(m.linear, t.rot) + broadcast(cross, t.trans, angular(:, i))
    return SpatialMotionMatrix(m.body, m.base, t.to, angular, linear)
end

immutable SpatialForceMatrix{N, T}
    body::RigidBody
    base::RigidBody
    frame::CartesianFrame3D
    angular::Mat{3, N, T}
    linear::Mat{3, N, T}
end
typealias SpatialForceVector{T} SpatialForceMatrix{1, T}

function transform{N, T}(f::SpatialForceMatrix{N, T}, t::Transform3D{T})
    @assert f.frame == t.from
    linear = rotate(f.linear, t.rot)
    angular = rotate(f.angular, t.rot) + broadcast(cross, t.trans, linear)
    return SpatialForceMatrix(f.body, f.base, t.to, angular, linear)
end

abstract Joint

immutable QuaternionFloatingJoint <: Joint
    name::ASCIIString
end
joint_transform{T}(j::QuaternionFloatingJoint, q::Vector{T}, before::CartesianFrame3D, after::CartesianFrame3D) = Transform3D(Quaternion(q[1], q[2 : 4]))
num_positions(joint::QuaternionFloatingJoint) = 7
num_velocities(joint::QuaternionFloatingJoint) = 6

immutable PrismaticJoint <: Joint
    name::ASCIIString
    translation_axis::Vec{3}
end
joint_transform{T}(j::PrismaticJoint, q::Vector{T}, before::CartesianFrame3D, after::CartesianFrame3D) = Transform3D(after, before, q[1] * j.axis.linear)
num_positions(joint::PrismaticJoint) = 1
num_velocities(joint::PrismaticJoint) = 1

immutable RevoluteJoint <: Joint
    name::ASCIIString
    rotation_axis::Vec{3}
end
joint_transform{T}(j::RevoluteJoint, q::Vector{T}, before::CartesianFrame3D, after::CartesianFrame3D) = Transform3D(after, before, qrotation(j.axis.angular, q[1]))
num_positions(joint::RevoluteJoint) = 1
num_velocities(joint::RevoluteJoint) = 1

# type Mechanism
#     graph::DiGraph
#     indexToJoint::Dict{Pair{Int64,Int64}, Joint}
#     jointToIndex::Dict{Joint, Pair{Int64,Int64}}
#     joints::Vector{Joint}
#     bodies::Vector{RigidBody}
#     bodyToIndex::Dict{RigidBody, Int64}

#     Mechanism() = new(DiGraph(), Dict(), Dict(), Vector(), Vector(), Dict())
# end

# function add_body!(m::Mechanism, b::RigidBody)
#     index = add_vertex!(m.graph)
#     push!(m.bodies, b)
#     m.bodyToIndex[b] = index
#     return b
# end

# function add_joint!(m::Mechanism, j::Joint, predecessor::RigidBody, successor::RigidBody)
#     index = add_edge!(m.graph, m.bodyToIndex[predecessor], m.bodyToIndex[successor])
#     push!(m.joints, j)
#     m.indexToJoint[index] = j
#     m.jointToIndex[j] = index
#     return j
# end

# type MechanismState{T}
#     q::Vector{T}
#     v::Vector{T}
# end
