# TODO: put in separate module

abstract type JointType{T} end
Base.eltype(::Type{JointType{T}}) where {T} = T

num_velocities(::T) where {T<:JointType} = num_velocities(T)
num_positions(::T) where {T<:JointType} = num_positions(T)
Base.@pure num_constraints(t::Type{T}) where {T<:JointType} = 6 - num_velocities(t)

# Default implementations
isfloating(::Type{<:JointType}) = false
flip_direction(jt::JointType{T}) where {T} = deepcopy(jt)

function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::JointType, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    ϕ .= q .- q0
    ϕ̇ .= v
    nothing
end

function global_coordinates!(q::AbstractVector, jt::JointType, q0::AbstractVector, ϕ::AbstractVector)
    q .= q0 .+ ϕ
end

function configuration_derivative_to_velocity_adjoint!(out, jt::JointType, q::AbstractVector, f)
    out .= f
end


"""
$(TYPEDEF)

A floating joint type that uses a unit quaternion representation for orientation.

Floating joints are 6-degree-of-freedom joints that are in a sense degenerate,
as they impose no constraints on the relative motion between two bodies.

The full, 7-dimensional configuration vector of a `QuaternionFloating` joint
type consists of a unit quaternion representing the orientation that rotates
vectors from the frame 'directly after' the joint to the frame 'directly before'
it, and a 3D position vector representing the origin of the frame after the
joint in the frame before the joint.

The 6-dimensional velocity vector of a `QuaternionFloating` joint is the twist
of the frame after the joint with respect to the frame before it, expressed in
the frame after the joint.
"""
struct QuaternionFloating{T} <: JointType{T}
end

Base.show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
Random.rand(::Type{QuaternionFloating{T}}) where {T} = QuaternionFloating{T}()

num_positions(::Type{<:QuaternionFloating}) = 7
num_velocities(::Type{<:QuaternionFloating}) = 6
isfloating(::Type{<:QuaternionFloating}) = true

@inline function rotation(jt::QuaternionFloating, q::AbstractVector, normalize::Bool = true)
    @inbounds quat = Quat(q[1], q[2], q[3], q[4], normalize)
    quat
end
@inline function rotation!(q::AbstractVector, jt::QuaternionFloating, rot::Rotation{3})
    quat = Quat(rot)
    @inbounds q[1] = quat.w
    @inbounds q[2] = quat.x
    @inbounds q[3] = quat.y
    @inbounds q[4] = quat.z
    nothing
end
@inline function rotation!(q::AbstractVector, jt::QuaternionFloating, rot::AbstractVector)
    @inbounds q[1] = rot[1]
    @inbounds q[2] = rot[2]
    @inbounds q[3] = rot[3]
    @inbounds q[4] = rot[4]
    nothing
end

@inline translation(jt::QuaternionFloating, q::AbstractVector) = @inbounds return SVector(q[5], q[6], q[7])
@inline translation!(q::AbstractVector, jt::QuaternionFloating, trans::AbstractVector) = @inbounds copy!(q, 5, trans, 1, 3)

@inline angular_velocity(jt::QuaternionFloating, v::AbstractVector) = @inbounds return SVector(v[1], v[2], v[3])
@inline angular_velocity!(v::AbstractVector, jt::QuaternionFloating, ω::AbstractVector) = @inbounds copy!(v, 1, ω, 1, 3)

@inline linear_velocity(jt::QuaternionFloating, v::AbstractVector) = @inbounds return SVector(v[4], v[5], v[6])
@inline linear_velocity!(v::AbstractVector, jt::QuaternionFloating, ν::AbstractVector) = @inbounds copy!(v, 4, ν, 1, 3)

function joint_transform(jt::QuaternionFloating, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector)
    Transform3D(frameAfter, frameBefore, rotation(jt, q), translation(jt, q))
end

function motion_subspace(jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(eye(SMatrix{3, 3, S}), zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), eye(SMatrix{3, 3, S}))
    GeometricJacobian(frameAfter, frameBefore, frameAfter, angular, linear)
end

function constraint_wrench_subspace(jt::QuaternionFloating{T}, jointTransform::Transform3D{<:AbstractMatrix{X}}) where {T, X}
    S = promote_type(T, X)
    WrenchMatrix(jointTransform.from, zeros(SMatrix{3, 0, S}), zeros(SMatrix{3, 0, S}))
end

function bias_acceleration(jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frameAfter, frameBefore, frameAfter)
end

has_fixed_subspaces(jt::QuaternionFloating) = true

function configuration_derivative_to_velocity!(v::AbstractVector, jt::QuaternionFloating, q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q)
    @inbounds quatdot = SVector(q̇[1], q̇[2], q̇[3], q̇[4])
    ω = angular_velocity_in_body(quat, quatdot)
    posdot = translation(jt, q̇)
    linear = inv(quat) * posdot
    angular_velocity!(v, jt, ω)
    linear_velocity!(v, jt, linear)
    nothing
end

function configuration_derivative_to_velocity_adjoint!(fq, jt::QuaternionFloating, q::AbstractVector, fv)
    quatnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2) # TODO: make this nicer
    quat = Quat(q[1] / quatnorm, q[2] / quatnorm, q[3] / quatnorm, q[4] / quatnorm, false)
    rot = (quat_derivative_to_body_angular_velocity_jacobian(quat)' * angular_velocity(jt, fv)) ./ quatnorm
    trans = quat * linear_velocity(jt, fv)
    rotation!(fq, jt, rot)
    translation!(fq, jt, trans)
    nothing
end

function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::QuaternionFloating, q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q)
    ω = angular_velocity(jt, v)
    linear = linear_velocity(jt, v)
    quatdot = quaternion_derivative(quat, ω)
    transdot = quat * linear
    rotation!(q̇, jt, quatdot)
    translation!(q̇, jt, transdot)
    nothing
end

function zero_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    rotation!(q, jt, eye(Quat{T}))
    translation!(q, jt, zeros(SVector{3, T}))
    nothing
end

function rand_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    rotation!(q, jt, rand(Quat{T}))
    translation!(q, jt, randn(SVector{3, T}))
    nothing
end

function joint_twist(jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = convert(SVector{3, S}, angular_velocity(jt, v))
    linear = convert(SVector{3, S}, linear_velocity(jt, v))
    Twist(frameAfter, frameBefore, frameAfter, angular, linear)
end

function joint_spatial_acceleration(jt::QuaternionFloating{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = convert(SVector{3, S}, angular_velocity(jt, vd))
    linear = convert(SVector{3, S}, linear_velocity(jt, vd))
    SpatialAcceleration(frameAfter, frameBefore, frameAfter, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::QuaternionFloating, q::AbstractVector, joint_wrench::Wrench)
    angular_velocity!(τ, jt, joint_wrench.angular)
    linear_velocity!(τ, jt, joint_wrench.linear)
    nothing
end

# uses exponential coordinates centered around q0
function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::QuaternionFloating, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    # anonymous helper frames
    frameBefore = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frameAfter = CartesianFrame3D()

    t0 = joint_transform(jt, frame0, frameBefore, q0) # 0 to before
    t = joint_transform(jt, frameAfter, frameBefore, q) # after to before
    relative_transform = inv(t0) * t # relative to q0
    twist = joint_twist(jt, frameAfter, frame0, q, v) # (q_0 is assumed not to change)
    ξ, ξ̇ = log_with_time_derivative(relative_transform, twist)

    @inbounds copy!(ϕ, 1, ξ.angular, 1, 3)
    @inbounds copy!(ϕ, 4, ξ.linear, 1, 3)

    @inbounds copy!(ϕ̇, 1, ξ̇.angular, 1, 3)
    @inbounds copy!(ϕ̇, 4, ξ̇.linear, 1, 3)

    nothing
end

function global_coordinates!(q::AbstractVector, jt::QuaternionFloating, q0::AbstractVector, ϕ::AbstractVector)
    # anonymous helper frames
    frameBefore = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frameAfter = CartesianFrame3D()

    t0 = joint_transform(jt, frame0, frameBefore, q0)
    @inbounds ξrot = SVector(ϕ[1], ϕ[2], ϕ[3])
    @inbounds ξtrans = SVector(ϕ[4], ϕ[5], ϕ[6])
    ξ = Twist(frameAfter, frame0, frame0, ξrot, ξtrans)
    relative_transform = exp(ξ)
    t = t0 * relative_transform
    rotation!(q, jt, rotation(t))
    translation!(q, jt, translation(t))
    nothing
end



#=
OneDegreeOfFreedomFixedAxis
=#
abstract type OneDegreeOfFreedomFixedAxis{T} <: JointType{T} end

num_positions(::Type{<:OneDegreeOfFreedomFixedAxis}) = 1
num_velocities(::Type{<:OneDegreeOfFreedomFixedAxis}) = 1

function zero_configuration!(q::AbstractVector, ::OneDegreeOfFreedomFixedAxis)
    q .= 0
    nothing
end

function rand_configuration!(q::AbstractVector, ::OneDegreeOfFreedomFixedAxis)
    randn!(q)
    nothing
 end

function bias_acceleration(jt::OneDegreeOfFreedomFixedAxis{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frameAfter, frameBefore, frameAfter)
end

has_fixed_subspaces(jt::OneDegreeOfFreedomFixedAxis) = true

function configuration_derivative_to_velocity!(v::AbstractVector, ::OneDegreeOfFreedomFixedAxis, q::AbstractVector, q̇::AbstractVector)
    v .= q̇
    nothing
end

function velocity_to_configuration_derivative!(q̇::AbstractVector, ::OneDegreeOfFreedomFixedAxis, q::AbstractVector, v::AbstractVector)
    q̇ .= v
    nothing
end


"""
$(TYPEDEF)

A `Prismatic` joint type allows translation along a fixed axis.
"""
struct Prismatic{T} <: OneDegreeOfFreedomFixedAxis{T}
    axis::SVector{3, T}
    rotationFromZAligned::RotMatrix3{T}

    """
    $(SIGNATURES)

    Construct a new `Prismatic` joint type, allowing translation along `axis`
    (expressed in the frame before the joint).
    """
    Prismatic(axis::AbstractVector{T}) where {T} = new{T}(axis, rotation_between(SVector(zero(T), zero(T), one(T)), SVector(axis)))
end

Base.show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.axis)")

function Random.rand(::Type{Prismatic{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Prismatic(axis)
end

flip_direction(jt::Prismatic) = Prismatic(-jt.axis)

function joint_transform(jt::Prismatic, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector)
    translation = q[1] * jt.axis
    Transform3D(frameAfter, frameBefore, translation)
end

function joint_twist(jt::Prismatic, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector, v::AbstractVector)
    linear = jt.axis * v[1]
    Twist(frameAfter, frameBefore, frameAfter, zeros(linear), linear)
end

function joint_spatial_acceleration(jt::Prismatic{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    linear = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frameAfter, frameBefore, frameAfter, zeros(linear), linear)
end

function motion_subspace(jt::Prismatic{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = zeros(SMatrix{3, 1, S})
    linear = SMatrix{3, 1, S}(jt.axis)
    GeometricJacobian(frameAfter, frameBefore, frameAfter, angular, linear)
end

function constraint_wrench_subspace(jt::Prismatic{T}, jointTransform::Transform3D{<:AbstractMatrix{X}}) where {T, X}
    S = promote_type(T, X)
    R = convert(RotMatrix3{S}, jt.rotationFromZAligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(R, zeros(SMatrix{3, 2, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), Rcols12)
    WrenchMatrix(jointTransform.from, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::Prismatic, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.linear, jt.axis)
    nothing
end


"""
$(TYPEDEF)

A `Revolute` joint type allows rotation about a fixed axis.
"""
struct Revolute{T} <: OneDegreeOfFreedomFixedAxis{T}
    axis::SVector{3, T}
    rotationFromZAligned::RotMatrix3{T}

    """
    $(SIGNATURES)

    Construct a new `Revolute` joint type, allowing rotation about `axis`
    (expressed in the frame before the joint).
    """
    Revolute(axis::AbstractVector{T}) where {T} = new{T}(axis, rotation_between(SVector(zero(T), zero(T), one(T)), SVector(axis)))
end

Base.show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.axis)")
function Random.rand(::Type{Revolute{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Revolute(axis)
end

flip_direction(jt::Revolute) = Revolute(-jt.axis)

function joint_transform(jt::Revolute, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D, q::AbstractVector)
    @inbounds aa = AngleAxis(q[1], jt.axis[1], jt.axis[2], jt.axis[3])
    Transform3D(frameAfter, frameBefore, convert(RotMatrix3{eltype(aa)}, aa))
end

function joint_twist(jt::Revolute, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    angular = jt.axis * v[1]
    Twist(frameAfter, frameBefore, frameAfter, angular, zeros(angular))
end

function joint_spatial_acceleration(jt::Revolute{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frameAfter, frameBefore, frameAfter, angular, zeros(angular))
end

function motion_subspace(jt::Revolute{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = SMatrix{3, 1, S}(jt.axis)
    linear = zeros(SMatrix{3, 1, S})
    GeometricJacobian(frameAfter, frameBefore, frameAfter, angular, linear)
end

function constraint_wrench_subspace(jt::Revolute{T}, jointTransform::Transform3D{<:AbstractMatrix{X}}) where {T, X}
    S = promote_type(T, X)
    R = convert(RotMatrix3{S}, jt.rotationFromZAligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(Rcols12, zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 2, S}), R)
    WrenchMatrix(jointTransform.from, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::Revolute, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(joint_wrench.angular, jt.axis)
    nothing
end


"""
$(TYPEDEF)

The `Fixed` joint type is a degenerate joint type, in the sense that it allows
no motion between its predecessor and successor rigid bodies.
"""
struct Fixed{T} <: JointType{T}
end
Base.show(io::IO, jt::Fixed) = print(io, "Fixed joint")
Random.rand(::Type{Fixed{T}}) where {T} = Fixed{T}()

num_positions(::Type{<:Fixed}) = 0
num_velocities(::Type{<:Fixed}) = 0

function joint_transform(jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    eye(Transform3DS{S}, frameAfter, frameBefore)
end

function joint_twist(jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(Twist{S}, frameAfter, frameBefore, frameAfter)
end

function joint_spatial_acceleration(jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    zero(SpatialAcceleration{S}, frameAfter, frameBefore, frameAfter)
end

function motion_subspace(jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    GeometricJacobian(frameAfter, frameBefore, frameAfter, zeros(SMatrix{3, 0, S}), zeros(SMatrix{3, 0, S}))
end

function constraint_wrench_subspace(jt::Fixed{T}, jointTransform::Transform3D{<:AbstractMatrix{X}}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(eye(SMatrix{3, 3, S}), zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), eye(SMatrix{3, 3, S}))
    WrenchMatrix(jointTransform.from, angular, linear)
end

zero_configuration!(q::AbstractVector, ::Fixed) = nothing
rand_configuration!(q::AbstractVector, ::Fixed) = nothing

function bias_acceleration(jt::Fixed{T}, frameAfter::CartesianFrame3D, frameBefore::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frameAfter, frameBefore, frameAfter)
end

has_fixed_subspaces(jt::Fixed) = true
configuration_derivative_to_velocity!(v::AbstractVector, ::Fixed, q::AbstractVector, q̇::AbstractVector) = nothing
velocity_to_configuration_derivative!(q̇::AbstractVector, ::Fixed, q::AbstractVector, v::AbstractVector) = nothing
joint_torque!(τ::AbstractVector, jt::Fixed, q::AbstractVector, joint_wrench::Wrench) = nothing
