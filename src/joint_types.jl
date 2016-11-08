# TODO: put in separate module

abstract JointType{T<:Real}
eltype{T}(::Union{JointType{T}, Type{JointType{T}}}) = T

# Default implementations
flip_direction{T}(jt::JointType{T}) = deepcopy(jt)

function _local_coordinates!(jt::JointType,
        ϕ::AbstractVector, ϕ̇::AbstractVector,
        q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    sub!(ϕ, q, q0)
    copy!(ϕ̇, v)
end

function _global_coordinates!(jt::JointType, q::AbstractVector, q0::AbstractVector, ϕ::AbstractVector)
    q .= q0 .+ ϕ # TODO: allocates on 0.5
end


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

function _joint_transform(
        jt::QuaternionFloating, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector)
    S = promote_type(eltype(jt), eltype(q))
    rot = convert(Quaternion{S}, rotation(jt, q))
    trans = convert(SVector{3, S}, translation(jt, q))
    Transform3D{S}(frameAfter, frameBefore, rot, trans)
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

function _local_coordinates!(jt::QuaternionFloating,
        ϕ::AbstractVector, ϕ̇::AbstractVector,
        q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    # references:
    # Murray, Richard M., et al.
    # A mathematical introduction to robotic manipulation.
    # CRC press, 1994.

    # Bullo, Francesco, and R. M. Murray.
    # "Proportional derivative (PD) control on the Euclidean group."
    # European Control Conference. Vol. 2. 1995.

    # use exponential coordinates centered around q0
    # proposition 2.9 in Murray et al.

    # anonymous helper frames
    frameBefore = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frameAfter = CartesianFrame3D()

    # compute transform from frame at q to frame at q0
    t0 = _joint_transform(jt, frame0, frameBefore, q0) # 0 to before
    t = _joint_transform(jt, frameAfter, frameBefore, q) # after to before
    relative_transform = inv(t0) * t
    rot = relative_transform.rot
    trans = relative_transform.trans

    # rotational part of local coordinates is simply the rotation vector corresponding to orientation relative to q0 frame:
    # not using angle_axis_proper because we want to reuse intermediate results
    Θ_over_2 = atan2(√(rot.v1^2 + rot.v2^2 + rot.v3^2), rot.s)
    Θ = 2 * Θ_over_2
    sΘ_over_2 = sin(Θ_over_2)
    cΘ_over_2 = cos(Θ_over_2)
    axis = Θ < eps(Θ) ? SVector(one(Θ), zero(Θ), zero(Θ)) : SVector(rot.v1, rot.v2, rot.v3) * (1 / sΘ_over_2)
    ϕrot = Θ * axis

    # translational part
    # see Bullo and Murray, (2.4) and (2.5)
    α = Θ_over_2 * cΘ_over_2 / sΘ_over_2 # TODO: singularity
    Θ_squared = Θ^2
    p = trans
    ϕtrans = p - 0.5 * ϕrot × p + (1 - α) / Θ_squared * ϕrot × (ϕrot × p) # Bullo, Murray, (2.5)

    # time derivatives of exponential coordinates
    # see Bullo and Murray, Lemma 4.
    # this is truely magic.
    ω = angular_velocity(jt, v)
    ν = linear_velocity(jt, v)
    β = Θ_over_2^2 / sΘ_over_2^2 # TODO: singularity
    A = (2 * (1 - α) + 0.5 * (α - β)) / Θ_squared
    B = ((1 - α) + 0.5 * (α - β)) / Θ_squared^2
    ϕ̇rot_cross_1, ϕ̇trans_cross_1 = se3_commutator(ϕrot, ϕtrans, ω, ν)
    ϕ̇rot_cross_2, ϕ̇trans_cross_2 = se3_commutator(ϕrot, ϕtrans, ϕ̇rot_cross_1, ϕ̇trans_cross_1)
    ϕ̇rot_cross_3, ϕ̇trans_cross_3 = se3_commutator(ϕrot, ϕtrans, ϕ̇rot_cross_2, ϕ̇trans_cross_2)
    ϕ̇rot_cross_4, ϕ̇trans_cross_4 = se3_commutator(ϕrot, ϕtrans, ϕ̇rot_cross_3, ϕ̇trans_cross_3)
    ϕ̇rot = ω + 0.5 * ϕ̇rot_cross_1 + A * ϕ̇rot_cross_2 + B * ϕ̇rot_cross_4
    ϕ̇trans = ν + 0.5 * ϕ̇trans_cross_1 + A * ϕ̇trans_cross_2 + B * ϕ̇trans_cross_4

    @inbounds copy!(ϕ, 1, ϕrot, 1, 3)
    @inbounds copy!(ϕ, 4, ϕtrans, 1, 3)

    @inbounds copy!(ϕ̇, 1, ϕ̇rot, 1, 3)
    @inbounds copy!(ϕ̇, 4, ϕ̇trans, 1, 3)

    # other implementations:
    # TODO: turn into tests
    # ϕtrans as derived in proposition 2.9 in Murray based on rotation matrices:
    # R = rotation_matrix(rot)
    # A = (eye(SMatrix{3, 3, T}) - R) * hat(axis) + axis * axis' * angle # prop 2.9 in Murray
    # ϕtrans = angle * (A \ trans)
    #
    # . See:
    # # Park, Jonghoon, and Wan-Kyun Chung.
    # # "Geometric integration on Euclidean group with application to articulated multibody systems."
    # # IEEE Transactions on Robotics 21.5 (2005): 850-863.
    # # Equations (23) and (24)
    #
    # # rotational part
    # ω = quaternion_floating_angular_velocity(v)
    # ϕ̇rot = rotation_vector_rate(ϕrot, ω)
    #
    # # translational part TODO: ugly, don't quite understand it
    # ν = quaternion_floating_linear_velocity(v)
    # ϕ̇trans = rotation_vector_rate(ϕrot, ν)
    # Θ = angle
    # Θ_2 = Θ / 2
    # s = sin(Θ_2)
    # c = cos(Θ_2)
    # β = s^2
    # γ = c / s # TODO: singularity?
    # D = (1 - γ) / Θ^2 * hat(ν, ω) + (1 / β + γ - 2) / Θ^4 * dot(ω, v) * hat_squared(ω)
    # ϕ̇trans += (D - 1/2 * hat(v)) * v

    nothing
end

function _global_coordinates!(jt::QuaternionFloating, q::AbstractVector, q0::AbstractVector, ϕ::AbstractVector)
    T = eltype(ϕ)

    # anonymous helper frames
    frameBefore = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frameAfter = CartesianFrame3D()

    # compute transform from frame at q to frame at q0
    t0 = _joint_transform(jt, frame0, frameBefore, q0)

    # exponentiate ϕ
    ϕrot = SVector{3}(view(ϕ, 1 : 3))
    ϕtrans = SVector{3}(view(ϕ, 4 : 6))
    Θ = norm(ϕrot)
    if Θ < eps(Θ)
        # 2.32 in Murray et al.
        rot = Quaternion{T}(one(T), zero(T), zero(T), zero(T), true)
        trans = ϕtrans
    else
        # 2.36 in Murray et al.
        rot = angle_axis_to_quaternion(Θ, ϕrot / Θ)
        # ω and v are not really velocities, but this is the notation used in 2.36
        ω = ϕrot / Θ
        v = ϕtrans / Θ
        trans = ω × v
        trans -= rotate(trans, rot)
        trans += ω * dot(ω, v) * Θ
    end
    relative_transform = Transform3D(frameAfter, frame0, rot, trans)
    t = t0 * relative_transform
    rotation!(jt, q, t.rot)
    translation!(jt, q, t.trans)
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
