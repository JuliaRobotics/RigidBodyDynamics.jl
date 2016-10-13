abstract JointType{T<:Real}
flip_direction{T}(jt::JointType{T}) = deepcopy(jt) # default behavior for flipping the direction of a joint

type Joint{T<:Real}
    name::String
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::JointType{T}

    Joint(name::String, jointType::JointType{T}) = new(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jointType)
end

Joint{T<:Real}(name::String, jointType::JointType{T}) = Joint{T}(name, jointType)

show(io::IO, joint::Joint) = print(io, "Joint \"$(joint.name)\": $(joint.jointType)")
showcompact(io::IO, joint::Joint) = print(io, "$(joint.name)")

num_positions(joint::Joint) = num_positions(joint.jointType)::Int64
num_velocities(joint::Joint) = num_velocities(joint.jointType)::Int64

num_positions(itr) = reduce((val, joint) -> val + num_positions(joint), 0, itr)
num_velocities(itr) = reduce((val, joint) -> val + num_velocities(joint), 0, itr)

function check_num_positions(joint::Joint, vec::AbstractVector)
    length(vec) == num_positions(joint) || error("wrong size")
    nothing
end

function check_num_velocities(joint::Joint, vec::AbstractVector)
    length(vec) == num_velocities(joint) || error("wrong size")
    nothing
end

# Return type annotations below because of https://groups.google.com/forum/#!topic/julia-users/OBs0fmNmjCU
# They might not be completely necessary at this point.
function joint_transform{M, X}(joint::Joint{M}, q::AbstractVector{X})::Transform3D{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    _joint_transform(joint, joint.jointType, q)
end

# TODO: currently not type stable (should probably return a fixed-maximum-size GeometricJacobian)
function motion_subspace(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    _motion_subspace(joint, joint.jointType, q)
end

function bias_acceleration{M, X}(joint::Joint{M}, q::AbstractVector{X}, v::AbstractVector{X})::SpatialAcceleration{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    _bias_acceleration(joint, joint.jointType, q, v)
end

function configuration_derivative_to_velocity!(joint::Joint, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)::Void
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q̇)
    _configuration_derivative_to_velocity!(joint.jointType, v, q, q̇)
end

function velocity_to_configuration_derivative!(joint::Joint, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q̇)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    _velocity_to_configuration_derivative!(joint.jointType, q̇, q, v)
end

function zero_configuration!(joint::Joint, q::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q)
    _zero_configuration!(joint.jointType, q)
end

function rand_configuration!(joint::Joint, q::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q)
    _rand_configuration!(joint.jointType, q)
end

function joint_twist{M, X}(joint::Joint{M}, q::AbstractVector{X}, v::AbstractVector{X})::Twist{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    _joint_twist(joint, joint.jointType, q, v)
end

function joint_torque!(joint::Joint, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)::Void
    @boundscheck check_num_velocities(joint, τ)
    @boundscheck check_num_positions(joint, q)
    framecheck(joint_wrench.frame, joint.frameAfter)
    _joint_torque!(joint.jointType, τ, q, joint_wrench)
end

immutable QuaternionFloating{T} <: JointType{T}
end

show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
rand{T}(::Type{QuaternionFloating{T}}) = QuaternionFloating{T}()

num_positions(::QuaternionFloating) = 7
num_velocities(::QuaternionFloating) = 6

function _joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @inbounds rot = Quaternion(q[1], q[2], q[3], q[4])
    Quaternions.normalize(rot)
    @inbounds trans = SVector{3}(q[5], q[6], q[7])
    Transform3D(j.frameAfter, j.frameBefore, convert(Quaternion{S}, rot), convert(SVector{3, S}, trans))
end

function _motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = hcat(eye(SMatrix{3, 3, S}), zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), eye(SMatrix{3, 3, S}))
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

function _bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function _configuration_derivative_to_velocity!(jt::QuaternionFloating, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    @inbounds quat = Quaternion(q[1], q[2], q[3], q[4])
    invquat = inv(quat)
    Quaternions.normalize(quat)
    @inbounds quatdot = Quaternion(q̇[1], q̇[2], q̇[3], q̇[4])
    @inbounds posdot = SVector{3}(q̇[5], q̇[6], q̇[7])
    linear = rotate(posdot, invquat)
    angularQuat = 2 * invquat * quatdot
    @inbounds v[1] = angularQuat.v1
    @inbounds v[2] = angularQuat.v2
    @inbounds v[3] = angularQuat.v3
    @inbounds v[4] = linear[1]
    @inbounds v[5] = linear[2]
    @inbounds v[6] = linear[3]
    nothing
end

function _velocity_to_configuration_derivative!(jt::QuaternionFloating, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    @inbounds quat = Quaternion(q[1], q[2], q[3], q[4])
    Quaternions.normalize(quat)
    @inbounds ωQuat = Quaternion(0, v[1], v[2], v[3])
    @inbounds linear = SVector{3}(v[4], v[5], v[6])
    quatdot = 1/2 * quat * ωQuat
    posdot = rotate(linear, quat)
    @inbounds q̇[1] = quatdot.s
    @inbounds q̇[2] = quatdot.v1
    @inbounds q̇[3] = quatdot.v2
    @inbounds q̇[4] = quatdot.v3
    @inbounds copy!(view(q̇, 5 : 7), posdot)
    nothing
end

function _zero_configuration!(jt::QuaternionFloating, q::AbstractVector)
    @inbounds q[1] = 1
    @inbounds fill!(view(q, 2 : 7), 0)
    nothing
end

function _rand_configuration!(jt::QuaternionFloating, q::AbstractVector)
    quat = nquatrand()
    @inbounds q[1] = quat.s
    @inbounds q[2] = quat.v1
    @inbounds q[3] = quat.v2
    @inbounds q[4] = quat.v3
    @inbounds randn!(view(q, 5 : 7))
    nothing
end

function _joint_twist{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    @inbounds ret = Twist(j.frameAfter, j.frameBefore, j.frameAfter, SVector{3, S}(v[1], v[2], v[3]), SVector{3, S}(v[4], v[5], v[6]))
    ret
end

function _joint_torque!(jt::QuaternionFloating, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @inbounds copy!(view(τ, 1 : 3), joint_wrench.angular)
    @inbounds copy!(view(τ, 4 : 6), joint_wrench.linear)
    nothing
end


abstract OneDegreeOfFreedomFixedAxis{T<:Real} <: JointType{T}

num_positions(::OneDegreeOfFreedomFixedAxis) = 1
num_velocities(::OneDegreeOfFreedomFixedAxis) = 1

function _zero_configuration!(::OneDegreeOfFreedomFixedAxis, q::AbstractVector)
    fill!(q, zero(eltype(q)))
    nothing
end

function _rand_configuration!(::OneDegreeOfFreedomFixedAxis, q::AbstractVector)
    randn!(q)
    nothing
 end

function _bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::OneDegreeOfFreedomFixedAxis{T}, q::AbstractVector{X}, v::AbstractVector{X})
    zero(SpatialAcceleration{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function _configuration_derivative_to_velocity!(::OneDegreeOfFreedomFixedAxis, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    v[:] = q̇
    nothing
end

function _velocity_to_configuration_derivative!(::OneDegreeOfFreedomFixedAxis, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    q̇[:] = v
    nothing
end


immutable Prismatic{T<:Real} <: OneDegreeOfFreedomFixedAxis{T}
    translation_axis::SVector{3, T}
end

show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.translation_axis)")
function rand{T}(::Type{Prismatic{T}})
    axis = rand(SVector{3, T})
    Prismatic(axis / norm(axis))
end

flip_direction(jt::Prismatic) = Prismatic(-jt.translation_axis)

function _joint_transform(j::Joint, jt::Prismatic, q::AbstractVector)
    @inbounds translation = q[1] * jt.translation_axis
    Transform3D(j.frameAfter, j.frameBefore, translation)
end

function _joint_twist(j::Joint, jt::Prismatic, q::AbstractVector, v::AbstractVector)
    @inbounds linear = jt.translation_axis * v[1]
    Twist(j.frameAfter, j.frameBefore, j.frameAfter, zeros(linear), linear)
end

function _motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Prismatic{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = zeros(SMatrix{3, 1, X})
    linear = SMatrix{3, 1, X}(jt.translation_axis)
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

function _joint_torque!(jt::Prismatic, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.linear, jt.translation_axis)
    nothing
end


immutable Revolute{T<:Real} <: OneDegreeOfFreedomFixedAxis{T}
    rotation_axis::SVector{3, T}
end

show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.rotation_axis)")
function rand{T}(::Type{Revolute{T}})
    axis = rand(SVector{3, T})
    Revolute(axis / norm(axis))
end

flip_direction(jt::Revolute) = Revolute(-jt.rotation_axis)

function _joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::Revolute{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @inbounds arg = q[1] / X(2)
    s = sin(arg)
    axis = jt.rotation_axis
    @inbounds rot = Quaternion(cos(arg), s * axis[1], s * axis[2], s * axis[3], true)
    Transform3D(j.frameAfter, j.frameBefore, rot)
end

function _joint_twist(j::Joint, jt::Revolute, q::AbstractVector, v::AbstractVector)
    @inbounds angular_velocity = jt.rotation_axis * v[1]
    Twist(j.frameAfter, j.frameBefore, j.frameAfter, angular_velocity, zeros(angular_velocity))
end

function _motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Revolute{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = SMatrix{3, 1, S}(jt.rotation_axis)
    linear = zeros(SMatrix{3, 1, S})
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

function _joint_torque!(jt::Revolute, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.angular, jt.rotation_axis)
    nothing
end

immutable Fixed{T<:Real} <: JointType{T}
end
show(io::IO, jt::Fixed) = print(io, "Fixed joint")
rand{T}(::Type{Fixed{T}}) = Fixed{T}()

num_positions(::Fixed) = 0
num_velocities(::Fixed) = 0

function _joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X})
    Transform3D(promote_type(T, X), j.frameAfter, j.frameBefore)
end

function _joint_twist{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X}, v::AbstractVector{X})
    zero(Twist{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function _motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, zeros(SMatrix{3, 0, S}), zeros(SMatrix{3, 0, S}))
end

_zero_configuration!(::Fixed, q::AbstractVector) = nothing
_rand_configuration!(::Fixed, q::AbstractVector) = nothing

function _bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X}, v::AbstractVector{X})
    zero(SpatialAcceleration{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

_configuration_derivative_to_velocity!(::Fixed, v::AbstractVector, q::AbstractVector, q̇::AbstractVector) = nothing
_velocity_to_configuration_derivative!(::Fixed, q̇::AbstractVector, q::AbstractVector, v::AbstractVector) = nothing
_joint_torque!(jt::Fixed, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench) = nothing
