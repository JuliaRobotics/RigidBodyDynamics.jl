type Joint{T <: JointType}
    name::String
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::T

    Joint(name::String, jointType::T) = new(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jointType)
end

Joint{T <: JointType}(name::String, jointType::T) = Joint{T}(name, jointType)

eltype{T}(::Type{Joint{T}}) = eltype(T)
show(io::IO, joint::Joint) = print(io, "Joint \"$(joint.name)\": $(joint.jointType)")
showcompact(io::IO, joint::Joint) = print(io, "$(joint.name)")

@inline function check_num_positions(joint::Joint, vec::AbstractVector)
    length(vec) == num_positions(joint) || error("wrong size")
    nothing
end

@inline function check_num_velocities(joint::Joint, vec::AbstractVector)
    length(vec) == num_velocities(joint) || error("wrong size")
    nothing
end

num_positions(joint::Joint) = num_positions(joint.jointType)
num_velocities(joint::Joint) = num_velocities(joint.jointType)

function joint_transform(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    joint_transform(joint.jointType, joint.frameAfter, joint.frameBefore, q)
end

function motion_subspace(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    motion_subspace(joint.jointType, joint.frameAfter, joint.frameBefore, q)
end

function bias_acceleration(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    bias_acceleration(joint.jointType, joint.frameAfter, joint.frameBefore, q, v)
end

function configuration_derivative_to_velocity!(joint::Joint, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q̇)
    configuration_derivative_to_velocity!(joint.jointType, v, q, q̇)
end

function velocity_to_configuration_derivative!(joint::Joint, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q̇)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    velocity_to_configuration_derivative!(joint.jointType, q̇, q, v)
end

function zero_configuration!(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    zero_configuration!(joint.jointType, q)
end

function rand_configuration!(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    rand_configuration!(joint.jointType, q)
end

function joint_twist(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    joint_twist(joint.jointType, joint.frameAfter, joint.frameBefore, q, v)
end

function joint_torque!(joint::Joint, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)
    @boundscheck check_num_velocities(joint, τ)
    @boundscheck check_num_positions(joint, q)
    @framecheck(joint_wrench.frame, joint.frameAfter)
    joint_torque!(joint.jointType, τ, q, joint_wrench)
end

function local_coordinates!(joint::Joint,
        ϕ::AbstractVector, ϕ̇::AbstractVector,
        q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_velocities(joint, ϕ)
    @boundscheck check_num_velocities(joint, ϕ̇)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    local_coordinates!(joint.jointType, ϕ, ϕ̇, q0, q, v)
end

function global_coordinates!(joint::Joint, q::AbstractVector, q0::AbstractVector, ϕ::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_velocities(joint, ϕ)
    global_coordinates!(joint.jointType, q, q0, ϕ)
end
