# TODO: put in separate module

abstract JointType{T<:Real}
flip_direction{T}(jt::JointType{T}) = deepcopy(jt) # default behavior for flipping the direction of a joint


#=
QuaternionFloating
=#
immutable QuaternionFloating{T} <: JointType{T}
end

show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
rand{T}(::Type{QuaternionFloating{T}}) = QuaternionFloating{T}()

num_positions(::QuaternionFloating) = 7
num_velocities(::QuaternionFloating) = 6

@inline function rotation(jt::QuaternionFloating, q::AbstractVector, normalized::Bool = true)
    @inbounds quat = Quaternion{eltype(q)}(q[1], q[2], q[3], q[4], normalized)
    quat
end
@inline function rotation!(jt::QuaternionFloating, q::AbstractVector, quat::Quaternion)
    @inbounds q[1] = quat.s
    @inbounds q[2] = quat.v1
    @inbounds q[3] = quat.v2
    @inbounds q[4] = quat.v3
    nothing
end

@inline translation(jt::QuaternionFloating, q::AbstractVector) = begin @inbounds trans = SVector(q[5], q[6], q[7]); trans end
@inline translation!(jt::QuaternionFloating, q::AbstractVector, trans::AbstractVector) = @inbounds copy!(q, 5, trans, 1, 3)

@inline angular_velocity(jt::QuaternionFloating, v::AbstractVector) = begin @inbounds ω = SVector(v[1], v[2], v[3]); ω end
@inline angular_velocity!(jt::QuaternionFloating, v::AbstractVector, ω::AbstractVector) = @inbounds copy!(v, 1, ω, 1, 3)

@inline linear_velocity(jt::QuaternionFloating, v::AbstractVector) = begin @inbounds ν = SVector(v[4], v[5], v[6]); ν end
@inline linear_velocity!(jt::QuaternionFloating, v::AbstractVector, ν::AbstractVector) = @inbounds copy!(v, 4, ν, 1, 3)

function _joint_transform{T<:Real, X<:Real}(
        jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    S = promote_type(T, X)
    rot = rotation(jt, q)
    Quaternions.normalize(rot)
    trans = translation(jt, q)
    Transform3D(frameAfter, frameBefore, convert(Quaternion{S}, rot), convert(SVector{3, S}, trans))
end

function _motion_subspace{T<:Real, X<:Real}(
        jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = hcat(eye(SMatrix{3, 3, S}), zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), eye(SMatrix{3, 3, S}))
    MotionSubspace(frameAfter, frameBefore, frameAfter, angular, linear)
end

function _bias_acceleration{T<:Real, X<:Real}(
        jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frameAfter, frameBefore, frameAfter)
end

function _configuration_derivative_to_velocity!(jt::QuaternionFloating, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q)
    invquat = inv(quat)
    quatdot = rotation(jt, q̇, false)
    posdot = translation(jt, q̇)
    linear = rotate(posdot, invquat)
    angularQuat = 2 * invquat * quatdot
    angular_velocity!(jt, v, SVector{3}(angularQuat.v1, angularQuat.v2, angularQuat.v3))
    linear_velocity!(jt, v, linear)
    nothing
end

function _velocity_to_configuration_derivative!(jt::QuaternionFloating, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q)
    ω = angular_velocity(jt, v)
    ωQuat = Quaternion(0, ω[1], ω[2], ω[3])
    linear = linear_velocity(jt, v)
    quatdot = 0.5 * quat * ωQuat
    transdot = rotate(linear, quat)
    rotation!(jt, q̇, quatdot)
    translation!(jt, q̇, transdot)
    nothing
end

function _zero_configuration!(jt::QuaternionFloating, q::AbstractVector)
    T = eltype(q)
    rotation!(jt, q, Quaternion(one(T), zero(T), zero(T), zero(T)))
    translation!(jt, q, zeros(SVector{3, T}))
    nothing
end

function _rand_configuration!(jt::QuaternionFloating, q::AbstractVector)
    T = eltype(q)
    rotation!(jt, q, nquatrand())
    translation!(jt, q, randn(SVector{3, T}))
    nothing
end

function _joint_twist{T<:Real, X<:Real}(
        jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    angular = convert(SVector{3, S}, angular_velocity(jt, v))
    linear = convert(SVector{3, S}, linear_velocity(jt, v))
    Twist(frameAfter, frameBefore, frameAfter, angular, linear)
end

function _joint_torque!(jt::QuaternionFloating, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    angular_velocity!(jt, τ, joint_wrench.angular)
    linear_velocity!(jt, τ, joint_wrench.linear)
    nothing
end


#=
OneDegreeOfFreedomFixedAxis
=#
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

function _bias_acceleration{T<:Real, X<:Real}(
        jt::OneDegreeOfFreedomFixedAxis{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X}, v::AbstractVector{X})
    zero(SpatialAcceleration{promote_type(T, X)}, frameAfter, frameBefore, frameAfter)
end

function _configuration_derivative_to_velocity!(::OneDegreeOfFreedomFixedAxis, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    v[:] = q̇
    nothing
end

function _velocity_to_configuration_derivative!(::OneDegreeOfFreedomFixedAxis, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    q̇[:] = v
    nothing
end


#=
Prismatic
=#
immutable Prismatic{T<:Real} <: OneDegreeOfFreedomFixedAxis{T}
    translation_axis::SVector{3, T}
end

show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.translation_axis)")
function rand{T}(::Type{Prismatic{T}})
    axis = rand(SVector{3, T})
    Prismatic(axis / norm(axis))
end

flip_direction(jt::Prismatic) = Prismatic(-jt.translation_axis)

function _joint_transform(
        jt::Prismatic, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector)
    @inbounds translation = q[1] * jt.translation_axis
    Transform3D(frameAfter, frameBefore, translation)
end

function _joint_twist(
        jt::Prismatic, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector, v::AbstractVector)
    @inbounds linear = jt.translation_axis * v[1]
    Twist(frameAfter, frameBefore, frameAfter, zeros(linear), linear)
end

function _motion_subspace{T<:Real, X<:Real}(
        jt::Prismatic{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = zeros(SMatrix{3, 1, X})
    linear = SMatrix{3, 1, X}(jt.translation_axis)
    MotionSubspace(frameAfter, frameBefore, frameAfter, angular, linear)
end

function _joint_torque!(jt::Prismatic, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.linear, jt.translation_axis)
    nothing
end


#=
Revolute
=#
immutable Revolute{T<:Real} <: OneDegreeOfFreedomFixedAxis{T}
    rotation_axis::SVector{3, T}
end

show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.rotation_axis)")
function rand{T}(::Type{Revolute{T}})
    axis = rand(SVector{3, T})
    Revolute(axis / norm(axis))
end

flip_direction(jt::Revolute) = Revolute(-jt.rotation_axis)

function _joint_transform{T<:Real, X<:Real}(
        jt::Revolute{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    @inbounds rot = angle_axis_to_quaternion(q[1], jt.rotation_axis)
    Transform3D(frameAfter, frameBefore, rot)
end

function _joint_twist(
        jt::Revolute, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector, v::AbstractVector)
    @inbounds angular_velocity = jt.rotation_axis * v[1]
    Twist(frameAfter, frameBefore, frameAfter, angular_velocity, zeros(angular_velocity))
end

function _motion_subspace{T<:Real, X<:Real}(
        jt::Revolute{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = SMatrix{3, 1, S}(jt.rotation_axis)
    linear = zeros(SMatrix{3, 1, S})
    MotionSubspace(frameAfter, frameBefore, frameAfter, angular, linear)
end

function _joint_torque!(jt::Revolute, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.angular, jt.rotation_axis)
    nothing
end


#=
Fixed
=#
immutable Fixed{T<:Real} <: JointType{T}
end
show(io::IO, jt::Fixed) = print(io, "Fixed joint")
rand{T}(::Type{Fixed{T}}) = Fixed{T}()

num_positions(::Fixed) = 0
num_velocities(::Fixed) = 0

function _joint_transform{T<:Real, X<:Real}(
        jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    Transform3D(promote_type(T, X), frameAfter, frameBefore)
end

function _joint_twist{T<:Real, X<:Real}(
        jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X}, v::AbstractVector{X})
    zero(Twist{promote_type(T, X)}, frameAfter, frameBefore, frameAfter)
end

function _motion_subspace{T<:Real, X<:Real}(
        jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X})
    S = promote_type(T, X)
    MotionSubspace(frameAfter, frameBefore, frameAfter, zeros(SMatrix{3, 0, S}), zeros(SMatrix{3, 0, S}))
end

_zero_configuration!(::Fixed, q::AbstractVector) = nothing
_rand_configuration!(::Fixed, q::AbstractVector) = nothing

function _bias_acceleration{T<:Real, X<:Real}(
        jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector{X}, v::AbstractVector{X})
    zero(SpatialAcceleration{promote_type(T, X)}, frameAfter, frameBefore, frameAfter)
end

_configuration_derivative_to_velocity!(::Fixed, v::AbstractVector, q::AbstractVector, q̇::AbstractVector) = nothing
_velocity_to_configuration_derivative!(::Fixed, q̇::AbstractVector, q::AbstractVector, v::AbstractVector) = nothing
_joint_torque!(jt::Fixed, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench) = nothing
