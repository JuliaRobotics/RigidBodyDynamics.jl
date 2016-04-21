abstract JointType

immutable Joint
    name::ASCIIString
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::JointType

    Joint(name::ASCIIString, jointType::JointType) = new(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jointType)
end

immutable QuaternionFloating <: JointType
    motionSubspaceAngular::Mat{3, 6, Float64}
    motionSubspaceLinear::Mat{3, 6, Float64}
    function QuaternionFloating()
        motionSubspaceAngular = Mat([eye(3) zeros(3, 3)])
        motionSubspaceLinear = Mat([zeros(3, 3) eye(3)])
        new(motionSubspaceAngular, motionSubspaceLinear)
    end
end

function joint_transform{T}(j::Joint, q::Vector{T}, jt::QuaternionFloating = j.jointType)
    rot = Quaternion(q[1], q[2 : 4])
    trans = Vec(q[5 : 7])
    return Transform3D{T}(j.frameAfter, j.frameBefore, rot, trans)
end

function motion_subspace{T}(j::Joint, q::Vector{T}, jt::QuaternionFloating = j.jointType)
    return MotionSubspaceBasis(j.frameBefore, j.frameAfter, j.frameAfter, jt.motionSubspaceAngular, jt.motionSubspaceLinear)
end

num_positions(j::Joint, jt::QuaternionFloating = j.jointType) = 7

num_velocities(j::Joint, jt::QuaternionFloating = j.jointType) = 6

function zero_configuration!{T}(j::Joint, v::Vector{T}, jt::QuaternionFloating = j.jointType)
    @assert length(v) == num_positions(j)
    v[1] = one(T)
    v[2 : end] = zero(T)
    return v
end

function joint_twist{T}(j::Joint, q::Vector{T}, v::Vector{T}, jt::QuaternionFloating = j.jointType)
    return Twist(joint.frameAfter, joint.frameBefore, joint.frameAfter, Vec(v(1 : 3)), Vec(v(4 : 6)))
end

immutable Prismatic{T} <: JointType
    translation_axis::Vec{3, T}
    motionSubspaceAngular::Mat{3, 1, T}
    motionSubspaceLinear::Mat{3, 1, T}
    Prismatic(translation_axis::Vec{3, T}) = new(translation_axis, zero(Mat{3, 1, T}), Mat(translation_axis))
end
Prismatic{T}(rotation_axis::Vec{3, T}) = Prismatic{T}(rotation_axis)

joint_transform{T1, T2}(j::Joint, q::Vector{T1}, jt::Prismatic{T2} = j.jointType) = Transform3D(j.frameAfter, j.frameBefore, q[1] * jt.translation_axis)

function joint_twist{T}(j::Joint, q::Vector{T}, v::Vector{T}, jt::Prismatic = j.jointType)
    return Twist(joint.frameAfter, joint.frameBefore, joint.frameAfter, zero(Vec{3, T}), jt.translation_axis * v[1])
end

function motion_subspace{T}(j::Joint, q::Vector{T}, jt::Prismatic = j.jointType)
    return MotionSubspaceBasis(j.frameBefore, j.frameAfter, j.frameAfter, jt.motionSubspaceAngular, jt.motionSubspaceLinear)
end

immutable Revolute{T} <: JointType
    rotation_axis::Vec{3, T}
    motionSubspaceAngular::Mat{3, 1, T}
    motionSubspaceLinear::Mat{3, 1, T}
    Revolute(rotation_axis::Vec{3, T}) = new(rotation_axis, Mat(rotation_axis), zero(Mat{3, 1, T}))
end
Revolute{T}(rotation_axis::Vec{3, T}) = Revolute{T}(rotation_axis)

joint_transform{T1, T2}(j::Joint, q::Vector{T1}, jt::Revolute{T2} = j.jointType) = Transform3D(j.frameAfter, j.frameBefore, qrotation(Array(jt.rotation_axis), q[1]))

function joint_twist{T}(j::Joint, q::Vector{T}, v::Vector{T}, jt::Revolute = j.jointType)
    return Twist(joint.frameAfter, joint.frameBefore, joint.frameAfter, jt.rotation_axis * v[1], zero(Vec{3, T}))
end

function motion_subspace{T}(j::Joint, q::Vector{T}, jt::Revolute = j.jointType)
    return MotionSubspaceBasis(j.frameBefore, j.frameAfter, j.frameAfter, jt.motionSubspaceAngular, jt.motionSubspaceLinear)
end

typealias OneDOF{T} Union{Prismatic{T}, Revolute{T}}
num_positions{T}(j::Joint, jt::OneDOF{T} = j.jointType) = 1
num_velocities{T}(j::Joint, jt::OneDOF{T} = j.jointType) = 1
function zero_configuration!{T1, T2}(j::Joint, v::Vector{T1}, jt::OneDOF{T2} = j.jointType)
    @assert length(v) == num_positions(j)
    v[1] = zero(T1)
    return v
end
