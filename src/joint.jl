abstract JointType{T<:Real}

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

"""
Given a function signature f(args...), generates a function
f(joint::Joint, args...) that delegates to a function
f(joint::Joint, jointType::JointType, args...)
which should be implemented by the implementor of a joint type
"""
macro joint_type_dependent_function(signature)
    functionName = signature.args[1]
    functionArgs = signature.args[2:end]
    eval(quote
        function $(functionName)(joint::Joint, $(functionArgs...))
            $(functionName)(joint, joint.jointType, $(functionArgs...))
        end
    end)
end

@joint_type_dependent_function joint_transform(q::AbstractVector)
@joint_type_dependent_function motion_subspace(q::AbstractVector)
@joint_type_dependent_function num_positions()
@joint_type_dependent_function num_velocities()
@joint_type_dependent_function bias_acceleration(q::AbstractVector, v::AbstractVector)
@joint_type_dependent_function configuration_derivative_to_velocity!(v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
@joint_type_dependent_function velocity_to_configuration_derivative!(q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
@joint_type_dependent_function zero_configuration!(q::AbstractVector)
@joint_type_dependent_function rand_configuration!(q::AbstractVector)
@joint_type_dependent_function joint_twist(q::AbstractVector, v::AbstractVector)
@joint_type_dependent_function joint_torque!(τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)

immutable QuaternionFloating{T} <: JointType{T}
end
show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
rand{T}(::Type{QuaternionFloating{T}}) = QuaternionFloating{T}()

function joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @inbounds rot = Quaternion(q[1], q[2], q[3], q[4])
    Quaternions.normalize(rot)
    @inbounds trans = SVector{3}(q[5], q[6], q[7])
    Transform3D(j.frameAfter, j.frameBefore, convert(Quaternion{S}, rot), convert(SVector{3, S}, trans))
end

function motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    angular = hcat(eye(SMatrix{3, 3, S}), zeros(SMatrix{3, 3, S}))
    linear = hcat(zeros(SMatrix{3, 3, S}), eye(SMatrix{3, 3, S}))
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

num_positions(j::Joint, jt::QuaternionFloating) = 7
num_velocities(j::Joint, jt::QuaternionFloating) = 6

function bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function configuration_derivative_to_velocity!(j::Joint, jt::QuaternionFloating, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
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

function velocity_to_configuration_derivative!(j::Joint, jt::QuaternionFloating, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
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
    @inbounds q̇[5 : 7] = posdot
    nothing
end

function zero_configuration!(j::Joint, jt::QuaternionFloating, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @inbounds q[1] = 1
    @inbounds fill!(view(q, 2 : 7), 0)
    nothing
end

function rand_configuration!(j::Joint, jt::QuaternionFloating, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    quat = nquatrand()
    @inbounds q[1] = quat.s
    @inbounds q[2] = quat.v1
    @inbounds q[3] = quat.v2
    @inbounds q[4] = quat.v3
    @inbounds randn!(view(q, 5 : 7))
    nothing
end

function joint_twist{T<:Real, X<:Real}(j::Joint{T}, jt::QuaternionFloating{T}, q::AbstractVector{X}, v::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @inbounds ret = Twist(j.frameAfter, j.frameBefore, j.frameAfter, SVector{3, S}(v[1], v[2], v[3]), SVector{3, S}(v[4], v[5], v[6]))
    ret
end

function joint_torque!(j::Joint, jt::QuaternionFloating, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    framecheck(joint_wrench.frame, j.frameAfter)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(τ) == num_velocities(j, jt) || error("τ has wrong size")
    @inbounds view(τ, 1 : 3)[:] = joint_wrench.angular
    @inbounds view(τ, 4 : 6)[:] = joint_wrench.linear
    nothing
end


abstract OneDegreeOfFreedomFixedAxis{T<:Real} <: JointType{T}

num_positions(j::Joint, jt::OneDegreeOfFreedomFixedAxis) = 1
num_velocities(j::Joint, jt::OneDegreeOfFreedomFixedAxis) = 1

function zero_configuration!(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    fill!(q, 0)
    nothing
end

function rand_configuration!(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    randn!(q)
    nothing
 end

function bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::OneDegreeOfFreedomFixedAxis{T}, q::AbstractVector{X}, v::AbstractVector{X})
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    zero(SpatialAcceleration{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function configuration_derivative_to_velocity!(j::Joint, jt::OneDegreeOfFreedomFixedAxis, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
    v[:] = q̇
end

function velocity_to_configuration_derivative!(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    q̇[:] = v
end


immutable Prismatic{T<:Real} <: OneDegreeOfFreedomFixedAxis{T}
    translation_axis::SVector{3, T}
end
# Prismatic{T}(rotation_axis::SVector{3, T}) = Prismatic{T}(rotation_axis)
show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.translation_axis)")
function rand{T}(::Type{Prismatic{T}})
    axis = rand(SVector{3, T})
    Prismatic(axis / norm(axis))
end

function joint_transform(j::Joint, jt::Prismatic, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @inbounds translation = q[1] * jt.translation_axis
    Transform3D(j.frameAfter, j.frameBefore, translation)
end

function joint_twist(j::Joint, jt::Prismatic, q::AbstractVector, v::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @inbounds linear = jt.translation_axis * v[1]
    Twist(j.frameAfter, j.frameBefore, j.frameAfter, zeros(linear), linear)
end

function motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Prismatic{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    angular = zeros(SMatrix{3, 1, X})
    linear = SMatrix{3, 1, X}(jt.translation_axis)
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

function joint_torque!(j::Joint, jt::Prismatic, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    framecheck(joint_wrench.frame, j.frameAfter)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(τ) == num_velocities(j, jt) || error("τ has wrong size")
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

function joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::Revolute{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @inbounds arg = q[1] / X(2)
    s = sin(arg)
    axis = jt.rotation_axis
    @inbounds rot = Quaternion(cos(arg), s * axis[1], s * axis[2], s * axis[3], true)
    Transform3D(j.frameAfter, j.frameBefore, rot)
end

function joint_twist(j::Joint, jt::Revolute, q::AbstractVector, v::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @inbounds angular_velocity = jt.rotation_axis * v[1]
    Twist(j.frameAfter, j.frameBefore, j.frameAfter, angular_velocity, zeros(angular_velocity))
end

function motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Revolute{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    angular = SMatrix{3, 1, S}(jt.rotation_axis)
    linear = zeros(SMatrix{3, 1, S})
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

function joint_torque!(j::Joint, jt::Revolute, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    framecheck(joint_wrench.frame, j.frameAfter)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(τ) == num_velocities(j, jt) || error("τ has wrong size")
    @inbounds τ[1] = dot(joint_wrench.angular, jt.rotation_axis)
    nothing
end

immutable Fixed{T<:Real} <: JointType{T}
end
show(io::IO, jt::Fixed) = print(io, "Fixed joint")
rand{T}(::Type{Fixed{T}}) = Fixed{T}()

function joint_transform{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X})
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    Transform3D(promote_type(T, X), j.frameAfter, j.frameBefore)
end

function joint_twist{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X}, v::AbstractVector{X})
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    zero(Twist{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function motion_subspace{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X})
    S = promote_type(T, X)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, zeros(SMatrix{3, 0, S}), zeros(SMatrix{3, 0, S}))
end

num_positions(j::Joint, jt::Fixed) = 0
num_velocities(j::Joint, jt::Fixed) = 0

function zero_configuration!(j::Joint, jt::Fixed, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    nothing
end

function rand_configuration!(j::Joint, jt::Fixed, q::AbstractVector)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    nothing
end

function bias_acceleration{T<:Real, X<:Real}(j::Joint{T}, jt::Fixed{T}, q::AbstractVector{X}, v::AbstractVector{X})
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    zero(SpatialAcceleration{promote_type(T, X)}, j.frameAfter, j.frameBefore, j.frameAfter)
end

function configuration_derivative_to_velocity!(j::Joint, jt::Fixed, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
    nothing
end

function velocity_to_configuration_derivative!(j::Joint, jt::Fixed, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck length(q̇) == num_positions(j, jt) || error("q̇ has wrong size")
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(v) == num_velocities(j, jt) || error("v has wrong size")
    nothing
end

function joint_torque!(j::Joint, jt::Fixed, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @boundscheck length(q) == num_positions(j, jt) || error("q has wrong size")
    @boundscheck length(τ) == num_velocities(j, jt) || error("τ has wrong size")
    nothing
end


num_positions(itr) = reduce((val, joint) -> val + num_positions(joint), 0, itr)
num_velocities(itr) = reduce((val, joint) -> val + num_velocities(joint), 0, itr)
