abstract JointType

immutable Joint
    name::String
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::JointType

    Joint(name::String, jointType::JointType) = new(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jointType)
end
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
@joint_type_dependent_function configuration_derivative_to_velocity(q::AbstractVector, q̇::AbstractVector)
@joint_type_dependent_function velocity_to_configuration_derivative(q::AbstractVector, v::AbstractVector)
@joint_type_dependent_function zero_configuration(t::Type)
@joint_type_dependent_function rand_configuration(t::Type)
@joint_type_dependent_function joint_twist(q::AbstractVector, v::AbstractVector)

immutable QuaternionFloating <: JointType
end
show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
rand(::Type{QuaternionFloating}) = QuaternionFloating()

function joint_transform{T<:Real}(j::Joint, jt::QuaternionFloating, q::AbstractVector{T})
    rot = Quaternion(q[1], q[2 : 4])
    Quaternions.normalize(rot)
    trans = Vec(q[5], q[6], q[7])
    return Transform3D{T}(j.frameAfter, j.frameBefore, rot, trans)
end

function motion_subspace{T<:Real}(j::Joint, jt::QuaternionFloating, q::AbstractVector{T})
    angular = hcat(eye(Mat{3, 3, T}), zero(Mat{3, 3, T}))
    linear = hcat(zero(Mat{3, 3, T}), eye(Mat{3, 3, T}))
    return GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, linear)
end

num_positions(j::Joint, jt::QuaternionFloating) = 7::Int64
num_velocities(j::Joint, jt::QuaternionFloating) = 6::Int64
bias_acceleration{T<:Real}(j::Joint, jt::QuaternionFloating, q::AbstractVector{T}, v::AbstractVector{T}) = zero(SpatialAcceleration{T}, j.frameAfter, j.frameBefore, j.frameAfter)

function configuration_derivative_to_velocity(j::Joint, jt::QuaternionFloating, q::AbstractVector, q̇::AbstractVector)
    quat = Quaternion(q[1], q[2 : 4])
    Quaternions.normalize(quat)
    quatdot = Quaternion(q̇[1], q̇[2 : 4])
    posdot = Vec(q̇[5], q̇[6], q̇[7])
    linear = rotate(posdot, inv(quat))
    angularQuat = 2 * inv(quat) * quatdot
    return [angularQuat.v1; angularQuat.v2; angularQuat.v3; linear...]
end

function velocity_to_configuration_derivative(j::Joint, jt::QuaternionFloating, q::AbstractVector, v::AbstractVector)
    quat = Quaternion(q[1], q[2 : 4])
    Quaternions.normalize(quat)
    ωQuat = Quaternion(0, v[1], v[2], v[3])
    linear = Vec(v[4], v[5], v[6])
    quatdot = 1/2 * quat * ωQuat
    posdot = rotate(linear, quat)
    return [quatdot.s; quatdot.v1; quatdot.v2; quatdot.v3; posdot...]
end

function zero_configuration{T<:Real}(j::Joint, jt::QuaternionFloating, ::Type{T})
    return [one(T); zeros(T, 6)]
end
function rand_configuration{T<:Real}(j::Joint, jt::QuaternionFloating, ::Type{T})
    quat = nquatrand() # TODO: only works when T == Float64
    return [quat.s; quat.v1; quat.v2; quat.v3; rand(T, 3)]
end

function joint_twist{T<:Real}(j::Joint, jt::QuaternionFloating, q::AbstractVector{T}, v::AbstractVector{T})
    return Twist(j.frameAfter, j.frameBefore, j.frameAfter, Vec(v[1], v[2], v[3]), Vec(v[4], v[5], v[6]))
end

abstract OneDegreeOfFreedomFixedAxis <: JointType

immutable Prismatic{T<:Real} <: OneDegreeOfFreedomFixedAxis
    translation_axis::Vec{3, T}
end
# Prismatic{T}(rotation_axis::Vec{3, T}) = Prismatic{T}(rotation_axis)
show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.translation_axis)")
rand{T}(::Type{Prismatic{T}}) = Prismatic(FixedSizeArrays.normalize(rand(Vec{3, T})))

joint_transform{T1<:Real, T2}(j::Joint, jt::Prismatic{T2}, q::AbstractVector{T1}) = Transform3D(j.frameAfter, j.frameBefore, q[1] * jt.translation_axis)

function joint_twist{T<:Real}(j::Joint, jt::Prismatic, q::AbstractVector{T}, v::AbstractVector{T})
    return Twist(j.frameAfter, j.frameBefore, j.frameAfter, zero(Vec{3, T}), jt.translation_axis * v[1])
end

function motion_subspace{T<:Real}(j::Joint, jt::Prismatic, q::AbstractVector{T})
    linear = Mat(jt.translation_axis)
    return GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, zero(linear), linear)
end

immutable Revolute{T<:Real} <: OneDegreeOfFreedomFixedAxis
    rotation_axis::Vec{3, T}
end
# Revolute{T}(rotation_axis::Vec{3, T}) = Revolute{T}(rotation_axis)
show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.rotation_axis)")
rand{T}(::Type{Revolute{T}}) = Revolute(FixedSizeArrays.normalize(rand(Vec{3, T})))

function joint_transform{T1, T2}(j::Joint, jt::Revolute{T2}, q::AbstractVector{T1})
    T = promote_type(T1, T2)
    arg = q[1] / T(2)
    s = sin(arg)
    axis = jt.rotation_axis
    rot = Quaternion(cos(arg), s * axis[1], s * axis[2], s * axis[3], true)
    Transform3D(j.frameAfter, j.frameBefore, rot)
end

function joint_twist{T<:Real}(j::Joint, jt::Revolute, q::AbstractVector{T}, v::AbstractVector{T})
    return Twist(j.frameAfter, j.frameBefore, j.frameAfter, jt.rotation_axis * v[1], zero(Vec{3, T}))
end

function motion_subspace{T<:Real}(j::Joint, jt::Revolute, q::AbstractVector{T})
    angular = Mat(jt.rotation_axis)
    return GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, angular, zero(angular))
end

num_positions(j::Joint, jt::OneDegreeOfFreedomFixedAxis) = 1::Int64
num_velocities(j::Joint, jt::OneDegreeOfFreedomFixedAxis) = 1::Int64
zero_configuration{T<:Real}(j::Joint, jt::OneDegreeOfFreedomFixedAxis, ::Type{T}) = [zero(T)]
rand_configuration{T<:Real}(j::Joint, jt::OneDegreeOfFreedomFixedAxis, ::Type{T}) = [rand(T)]
bias_acceleration{T<:Real}(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q::AbstractVector{T}, v::AbstractVector{T}) = zero(SpatialAcceleration{T}, j.frameAfter, j.frameBefore, j.frameAfter)
configuration_derivative_to_velocity(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q::AbstractVector, q̇::AbstractVector) = q̇
velocity_to_configuration_derivative(j::Joint, jt::OneDegreeOfFreedomFixedAxis, q::AbstractVector, v::AbstractVector) = v


immutable Fixed <: JointType
end
show(io::IO, jt::Fixed) = print(io, "Fixed joint")
rand(::Type{Fixed}) = Fixed()
joint_transform{T}(j::Joint, jt::Fixed, q::AbstractVector{T}) = Transform3D(T, j.frameAfter, j.frameBefore)
function joint_twist{T<:Real}(j::Joint, jt::Fixed, q::AbstractVector{T}, v::AbstractVector{T})
    return zero(Twist{T}, j.frameAfter, j.frameBefore, j.frameAfter)
end
function motion_subspace{T<:Real}(j::Joint, jt::Fixed, q::AbstractVector{T})
    return GeometricJacobian(j.frameAfter, j.frameBefore, j.frameAfter, Mat{3, 0, T}(), Mat{3, 0, T}())
end
num_positions(j::Joint, jt::Fixed) = 0::Int64
num_velocities(j::Joint, jt::Fixed) = 0::Int64
zero_configuration{T<:Real}(j::Joint, jt::Fixed, ::Type{T}) = zeros(T, 0, 1)
rand_configuration{T<:Real}(j::Joint, jt::Fixed, ::Type{T}) = zeros(T, 0, 1)
bias_acceleration{T<:Real}(j::Joint, jt::Fixed, q::AbstractVector{T}, v::AbstractVector{T}) = zero(SpatialAcceleration{T}, j.frameAfter, j.frameBefore, j.frameAfter)
configuration_derivative_to_velocity(j::Joint, jt::Fixed, q::AbstractVector, q̇::AbstractVector) = q̇
velocity_to_configuration_derivative(j::Joint, jt::Fixed, q::AbstractVector, v::AbstractVector) = v


num_positions(itr) = reduce((val, joint) -> val + num_positions(joint), 0, itr)
num_velocities(itr) = reduce((val, joint) -> val + num_velocities(joint), 0, itr)
