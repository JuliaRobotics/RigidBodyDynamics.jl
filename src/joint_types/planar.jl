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

    @doc """
    $(SIGNATURES)

    Construct a new `Planar` joint type with the ``xy``-plane in which translation is allowed defined
    by 3-vectors `x` and `y` expressed in the frame before the joint.
    """ ->
    function Planar{T}(x_axis::AbstractVector, y_axis::AbstractVector) where {T}
        x, y = map(axis -> normalize(SVector{3}(axis)), (x_axis, y_axis))
        @assert isapprox(x ⋅ y, 0; atol = 100 * eps(T))
        new{T}(x, y, x × y)
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

@propagate_inbounds function rand_configuration!(q::AbstractVector{T}, ::Planar) where {T}
    q[1] = rand() - T(0.5)
    q[2] = rand() - T(0.5)
    q[3] = randn()
    nothing
end

@propagate_inbounds function joint_transform(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    rot = RotMatrix(AngleAxis(q[3], jt.rot_axis[1], jt.rot_axis[2], jt.rot_axis[3], false))
    trans = jt.x_axis * q[1] + jt.y_axis * q[2]
    Transform3D(frame_after, frame_before, rot, trans)
end

@propagate_inbounds function joint_twist(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    angular = jt.rot_axis * v[3]
    linear = jt.x_axis * v[1] + jt.y_axis * v[2]
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_spatial_acceleration(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = jt.rot_axis * vd[3]
    linear = jt.x_axis * vd[1] + jt.y_axis * vd[2]
    SpatialAcceleration{S}(frame_after, frame_before, frame_after, angular, linear)
end

@inline function motion_subspace(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(zero(SMatrix{3, 2, S}), jt.rot_axis)
    linear = hcat(jt.x_axis, jt.y_axis, zero(SVector{3, S}))
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::Planar{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(zero(SVector{3, S}), jt.x_axis, jt.y_axis)
    linear = hcat(jt.rot_axis, zero(SMatrix{3, 2, S}))
    WrenchMatrix(joint_transform.from, angular, linear)
end

@inline function bias_acceleration(jt::Planar{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    zero(SpatialAcceleration{promote_type(T, X)}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::Planar, q::AbstractVector, joint_wrench::Wrench)
    τ[1] = dot(linear(joint_wrench), jt.x_axis)
    τ[2] = dot(linear(joint_wrench), jt.y_axis)
    τ[3] = dot(angular(joint_wrench), jt.rot_axis)
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, jt::Planar, q::AbstractVector, q̇::AbstractVector)
    vlinear = RotMatrix(-q[3]) * SVector(q̇[1], q̇[2])
    v[1] = vlinear[1]
    v[2] = vlinear[2]
    v[3] = q̇[3]
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::Planar, q::AbstractVector, v::AbstractVector)
    q̇linear = RotMatrix(q[3]) * SVector(v[1], v[2])
    q̇[1] = q̇linear[1]
    q̇[2] = q̇linear[2]
    q̇[3] = v[3]
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(out, jt::Planar, q::AbstractVector, f)
    outlinear = RotMatrix(q[3]) * SVector(f[1], f[2])
    out[1] = outlinear[1]
    out[2] = outlinear[2]
    out[3] = f[3]
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative_jacobian(::Planar, q::AbstractVector)
    # TODO: use SMatrix(RotZ(q[3]) once it's as fast
    rot = RotMatrix(q[3])
    @inbounds return @SMatrix([rot[1] rot[3] 0;
                               rot[2] rot[4] 0;
                               0         0   1])
end

@propagate_inbounds function configuration_derivative_to_velocity_jacobian(::Planar, q::AbstractVector)
    # TODO: use SMatrix(RotZ(-q[3]) once it's as fast
    rot = RotMatrix(-q[3])
    @inbounds return @SMatrix([rot[1] rot[3] 0;
                               rot[2] rot[4] 0;
                               0         0   1])
end
