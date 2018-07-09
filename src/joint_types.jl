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
struct QuaternionFloating{T} <: JointType{T} end

Base.show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
Random.rand(::Type{QuaternionFloating{T}}) where {T} = QuaternionFloating{T}()

num_positions(::Type{<:QuaternionFloating}) = 7
num_velocities(::Type{<:QuaternionFloating}) = 6
has_fixed_subspaces(jt::QuaternionFloating) = true
isfloating(::Type{<:QuaternionFloating}) = true

@inline function rotation(jt::QuaternionFloating, q::AbstractVector, normalize::Bool = true)
    @inbounds quat = Quat(q[1], q[2], q[3], q[4], normalize)
    quat
end

@inline function set_rotation!(q::AbstractVector, jt::QuaternionFloating, rot::Rotation{3, T}) where T
    quat = convert(Quat{T}, rot)
    @inbounds q[1] = quat.w
    @inbounds q[2] = quat.x
    @inbounds q[3] = quat.y
    @inbounds q[4] = quat.z
    nothing
end

@inline function set_rotation!(q::AbstractVector, jt::QuaternionFloating, rot::AbstractVector)
    @inbounds q[1] = rot[1]
    @inbounds q[2] = rot[2]
    @inbounds q[3] = rot[3]
    @inbounds q[4] = rot[4]
    nothing
end

@inline translation(jt::QuaternionFloating, q::AbstractVector) = @inbounds return SVector(q[5], q[6], q[7])
@inline set_translation!(q::AbstractVector, jt::QuaternionFloating, trans::AbstractVector) = @inbounds copyto!(q, 5, trans, 1, 3)

@inline angular_velocity(jt::QuaternionFloating, v::AbstractVector) = @inbounds return SVector(v[1], v[2], v[3])
@inline set_angular_velocity!(v::AbstractVector, jt::QuaternionFloating, ω::AbstractVector) = @inbounds copyto!(v, 1, ω, 1, 3)

@inline linear_velocity(jt::QuaternionFloating, v::AbstractVector) = @inbounds return SVector(v[4], v[5], v[6])
@inline set_linear_velocity!(v::AbstractVector, jt::QuaternionFloating, ν::AbstractVector) = @inbounds copyto!(v, 4, ν, 1, 3)

function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:QuaternionFloating}, config::Transform3D)
    check_num_positions(joint, q)
    @framecheck config.from frame_after(joint)
    @framecheck config.to frame_before(joint)
    set_rotation!(q, joint_type(joint), rotation(config))
    set_translation!(q, joint_type(joint), translation(config))
    q
end

function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:QuaternionFloating}, twist::Twist)
    check_num_velocities(joint, v)
    @framecheck twist.base frame_before(joint)
    @framecheck twist.body frame_after(joint)
    @framecheck twist.frame frame_after(joint)
    set_angular_velocity!(v, joint_type(joint), angular(twist))
    set_linear_velocity!(v, joint_type(joint), linear(twist))
    v
end

function joint_transform(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    Transform3D(frame_after, frame_before, rotation(jt, q), translation(jt, q))
end

function motion_subspace(jt::QuaternionFloating{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(one(SMatrix{3, 3, S}), zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), one(SMatrix{3, 3, S}))
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

function constraint_wrench_subspace(jt::QuaternionFloating{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    WrenchMatrix(joint_transform.from, zero(SMatrix{3, 0, S}), zero(SMatrix{3, 0, S}))
end

function bias_acceleration(jt::QuaternionFloating{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

function configuration_derivative_to_velocity!(v::AbstractVector, jt::QuaternionFloating, q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q)
    @inbounds quatdot = SVector(q̇[1], q̇[2], q̇[3], q̇[4])
    ω = angular_velocity_in_body(quat, quatdot)
    posdot = translation(jt, q̇)
    linear = inv(quat) * posdot
    set_angular_velocity!(v, jt, ω)
    set_linear_velocity!(v, jt, linear)
    nothing
end

function configuration_derivative_to_velocity_adjoint!(fq, jt::QuaternionFloating, q::AbstractVector, fv)
    quatnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2) # TODO: make this nicer
    quat = Quat(q[1] / quatnorm, q[2] / quatnorm, q[3] / quatnorm, q[4] / quatnorm, false)
    rot = (velocity_jacobian(angular_velocity_in_body, quat)' * angular_velocity(jt, fv)) ./ quatnorm
    trans = quat * linear_velocity(jt, fv)
    set_rotation!(fq, jt, rot)
    set_translation!(fq, jt, trans)
    nothing
end

function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::QuaternionFloating, q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q)
    ω = angular_velocity(jt, v)
    linear = linear_velocity(jt, v)
    quatdot = quaternion_derivative(quat, ω)
    transdot = quat * linear
    set_rotation!(q̇, jt, quatdot)
    set_translation!(q̇, jt, transdot)
    nothing
end


function velocity_to_configuration_derivative_jacobian(jt::QuaternionFloating, q::AbstractVector)
    quat = rotation(jt, q)
    vj = velocity_jacobian(quaternion_derivative, quat)
    R = RotMatrix(quat)
    # TODO: use hvcat once it's as fast
    @inbounds return @SMatrix([vj[1] vj[5] vj[9]  0    0    0;
                               vj[2] vj[6] vj[10] 0    0    0;
                               vj[3] vj[7] vj[11] 0    0    0;
                               vj[4] vj[8] vj[12] 0    0    0;
                               0     0     0      R[1] R[4] R[7];
                               0     0     0      R[2] R[5] R[8];
                               0     0     0      R[3] R[6] R[9]])
end

function configuration_derivative_to_velocity_jacobian(jt::QuaternionFloating, q::AbstractVector)
    quat = rotation(jt, q)
    vj = velocity_jacobian(angular_velocity_in_body, quat)
    R_inv = RotMatrix(inv(quat))
    # TODO: use hvcat once it's as fast
    @inbounds return @SMatrix([vj[1] vj[4] vj[7] vj[10] 0        0        0;
                               vj[2] vj[5] vj[8] vj[11] 0        0        0;
                               vj[3] vj[6] vj[9] vj[12] 0        0        0;
                               0     0     0     0      R_inv[1] R_inv[4] R_inv[7];
                               0     0     0     0      R_inv[2] R_inv[5] R_inv[8];
                               0     0     0     0      R_inv[3] R_inv[6] R_inv[9]])
end


function zero_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    set_rotation!(q, jt, one(Quat{T}))
    set_translation!(q, jt, zero(SVector{3, T}))
    nothing
end

function rand_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    set_rotation!(q, jt, rand(Quat{T}))
    set_translation!(q, jt, rand(SVector{3, T}) - 0.5)
    nothing
end

function joint_twist(jt::QuaternionFloating{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = convert(SVector{3, S}, angular_velocity(jt, v))
    linear = convert(SVector{3, S}, linear_velocity(jt, v))
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

function joint_spatial_acceleration(jt::QuaternionFloating{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = convert(SVector{3, S}, angular_velocity(jt, vd))
    linear = convert(SVector{3, S}, linear_velocity(jt, vd))
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::QuaternionFloating, q::AbstractVector, joint_wrench::Wrench)
    set_angular_velocity!(τ, jt, angular(joint_wrench))
    set_linear_velocity!(τ, jt, linear(joint_wrench))
    nothing
end

# uses exponential coordinates centered around q0
function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::QuaternionFloating, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    # anonymous helper frames # FIXME
    frame_before = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frame_after = CartesianFrame3D()

    quat0 = rotation(jt, q0, false)
    quat = rotation(jt, q, false)
    p0 = translation(jt, q0)
    p = translation(jt, q)

    quat0inv = inv(quat0)
    δquat = quat0inv * quat
    δp = quat0inv * (p - p0)
    relative_transform = Transform3D(frame_after, frame0, δquat, δp)

    twist = joint_twist(jt, frame_after, frame0, q, v) # (q_0 is assumed not to change)
    ξ, ξ̇ = log_with_time_derivative(relative_transform, twist)

    @inbounds copyto!(ϕ, 1, angular(ξ), 1, 3)
    @inbounds copyto!(ϕ, 4, linear(ξ), 1, 3)

    @inbounds copyto!(ϕ̇, 1, angular(ξ̇), 1, 3)
    @inbounds copyto!(ϕ̇, 4, linear(ξ̇), 1, 3)

    nothing
end

function global_coordinates!(q::AbstractVector, jt::QuaternionFloating, q0::AbstractVector, ϕ::AbstractVector)
    # anonymous helper frames #FIXME
    frame_before = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frame_after = CartesianFrame3D()

    t0 = joint_transform(jt, frame0, frame_before, q0)
    @inbounds ξrot = SVector(ϕ[1], ϕ[2], ϕ[3])
    @inbounds ξtrans = SVector(ϕ[4], ϕ[5], ϕ[6])
    ξ = Twist(frame_after, frame0, frame0, ξrot, ξtrans)
    relative_transform = exp(ξ)
    t = t0 * relative_transform
    set_rotation!(q, jt, rotation(t))
    set_translation!(q, jt, translation(t))
    nothing
end

normalize_configuration!(q::AbstractVector, jt::QuaternionFloating) = set_rotation!(q, jt, rotation(jt, q, true))

function is_configuration_normalized(jt::QuaternionFloating, q::AbstractVector, rtol, atol)
    isapprox(quatnorm(rotation(jt, q, false)), one(eltype(q)); rtol = rtol, atol = atol)
end


#=
OneDegreeOfFreedomFixedAxis
=#
abstract type OneDegreeOfFreedomFixedAxis{T} <: JointType{T} end

num_positions(::Type{<:OneDegreeOfFreedomFixedAxis}) = 1
num_velocities(::Type{<:OneDegreeOfFreedomFixedAxis}) = 1
has_fixed_subspaces(jt::OneDegreeOfFreedomFixedAxis) = true
isfloating(::Type{<:OneDegreeOfFreedomFixedAxis}) = false

function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:OneDegreeOfFreedomFixedAxis}, config::Number)
    check_num_positions(joint, q)
    q[1] = config
    q
end

function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:OneDegreeOfFreedomFixedAxis}, vel::Number)
    check_num_positions(joint, v)
    v[1] = vel
    v
end

function rand_configuration!(q::AbstractVector, ::OneDegreeOfFreedomFixedAxis)
    randn!(q)
    nothing
 end

function bias_acceleration(jt::OneDegreeOfFreedomFixedAxis{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

function velocity_to_configuration_derivative_jacobian(::OneDegreeOfFreedomFixedAxis{T}, ::AbstractVector) where T
    @SMatrix([one(T)])
end

function configuration_derivative_to_velocity_jacobian(::OneDegreeOfFreedomFixedAxis{T}, ::AbstractVector) where T
    @SMatrix([one(T)])
end

"""
$(TYPEDEF)

A `Prismatic` joint type allows translation along a fixed axis.
"""
struct Prismatic{T} <: OneDegreeOfFreedomFixedAxis{T}
    axis::SVector{3, T}
    rotation_from_z_aligned::RotMatrix3{T}

    """
    $(SIGNATURES)

    Construct a new `Prismatic` joint type, allowing translation along `axis`
    (expressed in the frame before the joint).
    """
    function Prismatic(axis::AbstractVector{T}) where {T}
        a = normalize(axis)
        new{T}(a, rotation_between(SVector(zero(T), zero(T), one(T)), SVector{3, T}(a)))
    end
end

Base.show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.axis)")

function Random.rand(::Type{Prismatic{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Prismatic(axis)
end

RigidBodyDynamics.flip_direction(jt::Prismatic) = Prismatic(-jt.axis)

function joint_transform(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    translation = q[1] * jt.axis
    Transform3D(frame_after, frame_before, translation)
end

function joint_twist(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector, v::AbstractVector)
    linear = jt.axis * v[1]
    Twist(frame_after, frame_before, frame_after, zero(linear), linear)
end

function joint_spatial_acceleration(jt::Prismatic{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    linear = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frame_after, frame_before, frame_after, zero(linear), linear)
end

function motion_subspace(jt::Prismatic{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = zero(SMatrix{3, 1, S})
    linear = SMatrix{3, 1, S}(jt.axis)
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

function constraint_wrench_subspace(jt::Prismatic{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    R = convert(RotMatrix3{S}, jt.rotation_from_z_aligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(R, zero(SMatrix{3, 2, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), Rcols12)
    WrenchMatrix(joint_transform.from, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::Prismatic, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(linear(joint_wrench), jt.axis)
    nothing
end


"""
$(TYPEDEF)

A `Revolute` joint type allows rotation about a fixed axis.
"""
struct Revolute{T} <: OneDegreeOfFreedomFixedAxis{T}
    axis::SVector{3, T}
    rotation_from_z_aligned::RotMatrix3{T}

    """
    $(SIGNATURES)

    Construct a new `Revolute` joint type, allowing rotation about `axis`
    (expressed in the frame before the joint).
    """
    function Revolute(axis::AbstractVector{T}) where {T}
        a = normalize(axis)
        new{T}(a, rotation_between(SVector(zero(T), zero(T), one(T)), SVector{3, T}(a)))
    end
end

Base.show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.axis)")
function Random.rand(::Type{Revolute{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Revolute(axis)
end

RigidBodyDynamics.flip_direction(jt::Revolute) = Revolute(-jt.axis)

function joint_transform(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    @inbounds aa = AngleAxis(q[1], jt.axis[1], jt.axis[2], jt.axis[3], false)
    Transform3D(frame_after, frame_before, convert(RotMatrix3{eltype(aa)}, aa))
end

function joint_twist(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    angular = jt.axis * v[1]
    Twist(frame_after, frame_before, frame_after, angular, zero(angular))
end

function joint_spatial_acceleration(jt::Revolute{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, zero(angular))
end

function motion_subspace(jt::Revolute{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = SMatrix{3, 1, S}(jt.axis)
    linear = zero(SMatrix{3, 1, S})
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

function constraint_wrench_subspace(jt::Revolute{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    R = convert(RotMatrix3{S}, jt.rotation_from_z_aligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(Rcols12, zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 2, S}), R)
    WrenchMatrix(joint_transform.from, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::Revolute, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(angular(joint_wrench), jt.axis)
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
RigidBodyDynamics.flip_direction(jt::Fixed) = deepcopy(jt)

num_positions(::Type{<:Fixed}) = 0
num_velocities(::Type{<:Fixed}) = 0
has_fixed_subspaces(jt::Fixed) = true
isfloating(::Type{<:Fixed}) = false

function joint_transform(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    eye(Transform3D{S}, frame_after, frame_before)
end

function joint_twist(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(Twist{S}, frame_after, frame_before, frame_after)
end

function joint_spatial_acceleration(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

function motion_subspace(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    GeometricJacobian(frame_after, frame_before, frame_after, zero(SMatrix{3, 0, S}), zero(SMatrix{3, 0, S}))
end

function constraint_wrench_subspace(jt::Fixed{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(one(SMatrix{3, 3, S}), zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), one(SMatrix{3, 3, S}))
    WrenchMatrix(joint_transform.from, angular, linear)
end

zero_configuration!(q::AbstractVector, ::Fixed) = nothing
rand_configuration!(q::AbstractVector, ::Fixed) = nothing

function bias_acceleration(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

configuration_derivative_to_velocity!(v::AbstractVector, ::Fixed, q::AbstractVector, q̇::AbstractVector) = nothing
velocity_to_configuration_derivative!(q̇::AbstractVector, ::Fixed, q::AbstractVector, v::AbstractVector) = nothing
joint_torque!(τ::AbstractVector, jt::Fixed, q::AbstractVector, joint_wrench::Wrench) = nothing

function velocity_to_configuration_derivative_jacobian(::Fixed{T}, ::AbstractVector) where T
    SMatrix{0, 0, T}()
end

function configuration_derivative_to_velocity_jacobian(::Fixed{T}, ::AbstractVector) where T
    SMatrix{0, 0, T}()
end


"""
$(TYPEDEF)

The `Planar` joint type allows translation along two orthogonal vectors, referred to as ``x`` and ``y``,
as well as rotation about an axis ``z = x \\times y``.

The components of the 3-dimensional configuration vector ``q`` associated with a `Planar` joint are the
``x``- and ``y``-coordinates of the translation, and the angle of rotation ``\\theta`` about ``z``, in that order.

The components of the 3-dimension velocity vector ``v`` associated with a `Planar` joint are the
``x``- and ``y``-coordinates of the linear part of the joint twist, expressed in the frame after the joint,
followed by the ``z``-component of the angular part of this joint twist.

!!! warning

    For the `Planar` joint type, ``v \\neq \\dot{q}``! Although the angular parts of ``v`` and ``\\dot{q}``
    are the same, their linear parts differ. The linear part of ``v`` is the linear part of ``\\dot{q}``, rotated
    to the frame after the joint. This parameterization was chosen to allow the translational component of the
    joint transform to be independent of the rotation angle ``\\theta`` (i.e., the rotation is applied **after** the
    translation), while still retaining a constant motion subspace expressed in the frame after the joint.

"""
struct Planar{T} <: JointType{T}
    x_axis::SVector{3, T}
    y_axis::SVector{3, T}
    rot_axis::SVector{3, T}

    """
    $(SIGNATURES)

    Construct a new `Planar` joint type with the ``xy``-plane in which translation is allowed defined
    by 3-vectors `x` and `y` expressed in the frame before the joint.
    """
    function Planar{T}(x_axis::AbstractVector, y_axis::AbstractVector) where {T}
        x, y = map(axis -> normalize(SVector{3}(axis)), (x_axis, y_axis))
        @assert isapprox(x ⋅ y, 0; atol = 100 * eps(T))
        z = cross(x, y)
        new{T}(x, y, z)
    end
end

Planar(x_axis::AbstractVector{X}, y_axis::AbstractVector{Y}) where {X, Y} = Planar{promote_type(X, Y)}(x_axis, y_axis)

Base.show(io::IO, jt::Planar) = print(io, "Planar joint with x-axis $(jt.x_axis) and y-axis $(jt.y_axis)")

function Random.rand(::Type{Planar{T}}) where {T}
    x = normalize(randn(SVector{3, T}))
    y = normalize(randn(SVector{3, T}))
    y = normalize(y - (x ⋅ y) * x)
    Planar(x, y)
end

num_positions(::Type{<:Planar}) = 3
num_velocities(::Type{<:Planar}) = 3
has_fixed_subspaces(jt::Planar) = true
isfloating(::Type{<:Planar}) = false

function rand_configuration!(q::AbstractVector{T}, ::Planar) where {T}
    q[1] = rand() - T(0.5)
    q[2] = rand() - T(0.5)
    q[3] = randn()
    nothing
end

function joint_transform(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    @inbounds rot = RotMatrix(AngleAxis(q[3], jt.rot_axis[1], jt.rot_axis[2], jt.rot_axis[3], false))
    @inbounds trans = jt.x_axis * q[1] + jt.y_axis * q[2]
    Transform3D(frame_after, frame_before, rot, trans)
end

function joint_twist(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    @inbounds angular = jt.rot_axis * v[3]
    @inbounds linear = jt.x_axis * v[1] + jt.y_axis * v[2]
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

function joint_spatial_acceleration(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    @inbounds angular = jt.rot_axis * vd[3]
    @inbounds linear = jt.x_axis * vd[1] + jt.y_axis * vd[2]
    SpatialAcceleration{S}(frame_after, frame_before, frame_after, angular, linear)
end

function motion_subspace(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(zero(SMatrix{3, 2, S}), jt.rot_axis)
    linear = hcat(jt.x_axis, jt.y_axis, zero(SVector{3, S}))
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

function constraint_wrench_subspace(jt::Planar{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(zero(SVector{3, S}), jt.x_axis, jt.y_axis)
    linear = hcat(jt.rot_axis, zero(SMatrix{3, 2, S}))
    WrenchMatrix(joint_transform.from, angular, linear)
end

function bias_acceleration(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    zero(SpatialAcceleration{promote_type(T, X)}, frame_after, frame_before, frame_after)
end

function joint_torque!(τ::AbstractVector, jt::Planar, q::AbstractVector, joint_wrench::Wrench)
    @inbounds τ[1] = dot(linear(joint_wrench), jt.x_axis)
    @inbounds τ[2] = dot(linear(joint_wrench), jt.y_axis)
    @inbounds τ[3] = dot(angular(joint_wrench), jt.rot_axis)
    nothing
end

function configuration_derivative_to_velocity!(v::AbstractVector, jt::Planar, q::AbstractVector, q̇::AbstractVector)
    vlinear = RotMatrix(-q[3]) * SVector(q̇[1], q̇[2])
    v[1] = vlinear[1]
    v[2] = vlinear[2]
    v[3] = q̇[3]
    nothing
end

function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::Planar, q::AbstractVector, v::AbstractVector)
    q̇linear = RotMatrix(q[3]) * SVector(v[1], v[2])
    q̇[1] = q̇linear[1]
    q̇[2] = q̇linear[2]
    q̇[3] = v[3]
    nothing
end

function configuration_derivative_to_velocity_adjoint!(out, jt::Planar, q::AbstractVector, f)
    outlinear = RotMatrix(q[3]) * SVector(f[1], f[2])
    out[1] = outlinear[1]
    out[2] = outlinear[2]
    out[3] = f[3]
    nothing
end

function velocity_to_configuration_derivative_jacobian(::Planar, q::AbstractVector)
    # TODO: use SMatrix(RotZ(q[3]) once it's as fast
    rot = RotMatrix(q[3])
    @inbounds return @SMatrix([rot[1] rot[3] 0;
                               rot[2] rot[4] 0;
                               0         0   1])
end

function configuration_derivative_to_velocity_jacobian(::Planar, q::AbstractVector)
    # TODO: use SMatrix(RotZ(-q[3]) once it's as fast
    rot = RotMatrix(-q[3])
    @inbounds return @SMatrix([rot[1] rot[3] 0;
                               rot[2] rot[4] 0;
                               0         0   1])
end



"""
$(TYPEDEF)

The `QuaternionSpherical` joint type allows rotation in any direction. It is an
implementation of a ball-and-socket joint.

The 4-dimensional configuration vector ``q`` associated with a `QuaternionSpherical` joint
is the unit quaternion that describes the orientation of the frame after the joint
with respect to the frame before the joint. In other words, it is the quaternion that
can be used to rotate vectors from the frame after the joint to the frame before the
joint.

The 3-dimensional velocity vector ``v`` associated with a `QuaternionSpherical` joint is
the angular velocity of the frame after the joint with respect to the frame before
the joint, expressed in the frame after the joint (body frame).
"""
struct QuaternionSpherical{T} <: JointType{T} end

Base.show(io::IO, jt::QuaternionSpherical) = print(io, "Quaternion spherical joint")
Random.rand(::Type{QuaternionSpherical{T}}) where {T} = QuaternionSpherical{T}()
num_positions(::Type{<:QuaternionSpherical}) = 4
num_velocities(::Type{<:QuaternionSpherical}) = 3
has_fixed_subspaces(jt::QuaternionSpherical) = true
isfloating(::Type{<:QuaternionSpherical}) = false

@inline function rotation(jt::QuaternionSpherical, q::AbstractVector, normalize::Bool = true)
    @inbounds quat = Quat(q[1], q[2], q[3], q[4], normalize)
    quat
end

@inline function set_rotation!(q::AbstractVector, jt::QuaternionSpherical, rot::Rotation{3, T}) where {T}
    quat = convert(Quat{T}, rot)
    @inbounds q[1] = quat.w
    @inbounds q[2] = quat.x
    @inbounds q[3] = quat.y
    @inbounds q[4] = quat.z
    nothing
end

function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:QuaternionSpherical}, rot::Rotation{3})
    check_num_positions(joint, q)
    set_rotation!(q, joint_type(joint), rot)
    q
end

function joint_transform(jt::QuaternionSpherical, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    quat = rotation(jt, q)
    @inbounds return Transform3D(frame_after, frame_before, quat)
end

function motion_subspace(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = one(SMatrix{3, 3, S})
    linear = zero(SMatrix{3, 3, S})
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

function constraint_wrench_subspace(jt::QuaternionSpherical{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = zero(SMatrix{3, 3, S})
    linear = one(SMatrix{3, 3, S})
    WrenchMatrix(joint_transform.from, angular, linear)
end

function bias_acceleration(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

function configuration_derivative_to_velocity!(v::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q)
    @inbounds quatdot = SVector(q̇[1], q̇[2], q̇[3], q̇[4])
    v .= angular_velocity_in_body(quat, quatdot)
    nothing
end

function configuration_derivative_to_velocity_adjoint!(fq, jt::QuaternionSpherical, q::AbstractVector, fv)
    quatnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2) # TODO: make this nicer
    quat = Quat(q[1] / quatnorm, q[2] / quatnorm, q[3] / quatnorm, q[4] / quatnorm, false)
    fq .= (velocity_jacobian(angular_velocity_in_body, quat)' * fv) ./ quatnorm
    nothing
end

function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q)
    q̇ .= quaternion_derivative(quat, v)
    nothing
end

function velocity_to_configuration_derivative_jacobian(jt::QuaternionSpherical, q::AbstractVector)
    quat = rotation(jt, q)
    velocity_jacobian(quaternion_derivative, quat)
end

function configuration_derivative_to_velocity_jacobian(jt::QuaternionSpherical, q::AbstractVector)
    quat = rotation(jt, q)
    velocity_jacobian(angular_velocity_in_body, quat)
end

function zero_configuration!(q::AbstractVector, jt::QuaternionSpherical)
    T = eltype(q)
    set_rotation!(q, jt, one(Quat{T}))
    nothing
end

function rand_configuration!(q::AbstractVector, jt::QuaternionSpherical)
    T = eltype(q)
    set_rotation!(q, jt, rand(Quat{T}))
    nothing
end

function joint_twist(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = SVector{3, S}(v)
    linear = zero(SVector{3, S})
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

function joint_spatial_acceleration(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = SVector{3, S}(vd)
    linear = zero(SVector{3, S})
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, linear)
end

function joint_torque!(τ::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, joint_wrench::Wrench)
    τ .= angular(joint_wrench)
    nothing
end

# uses exponential coordinates centered around q0
function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::QuaternionSpherical, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    quat = inv(rotation(jt, q0)) * rotation(jt, q)
    rv = RodriguesVec(quat)
    ϕstatic = SVector(rv.sx, rv.sy, rv.sz)
    ϕ .= ϕstatic
    ϕ̇ .= rotation_vector_rate(ϕstatic, v)
    nothing
end

function global_coordinates!(q::AbstractVector, jt::QuaternionSpherical, q0::AbstractVector, ϕ::AbstractVector)
    quat0 = rotation(jt, q0)
    quat = quat0 * Quat(RodriguesVec(ϕ[1], ϕ[2], ϕ[3]))
    set_rotation!(q, jt, quat)
    nothing
end

normalize_configuration!(q::AbstractVector, jt::QuaternionSpherical) = set_rotation!(q, jt, rotation(jt, q, true))

function is_configuration_normalized(jt::QuaternionSpherical, q::AbstractVector, rtol, atol)
    isapprox(quatnorm(rotation(jt, q, false)), one(eltype(q)); rtol = rtol, atol = atol)
end
