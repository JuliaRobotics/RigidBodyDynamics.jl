module PDControl

using RigidBodyDynamics.Spatial
using StaticArrays
using Rotations

# functions
export
    pd

# gain types
export
    PDGains,
    FramePDGains,
    SE3PDGains

# pd control methods
export
    SE3PDMethod

abstract type AbstractPDGains end

group_error(x, xdes) = x - xdes
group_error(x::Rotation, xdes::Rotation) = inv(xdes) * x
group_error(x::Transform3D, xdes::Transform3D) = inv(xdes) * x

pd(gains::AbstractPDGains, x, xdes, ẋ, ẋdes) = pd(gains, group_error(x, xdes), -ẋdes + ẋ)
pd(gains::AbstractPDGains, x, xdes, ẋ, ẋdes, method) = pd(gains, group_error(x, xdes), -ẋdes + ẋ, method)

struct PDGains{K, D} <: AbstractPDGains
    k::K
    d::D
end

pd(gains::PDGains, e, ė) = -gains.k * e - gains.d * ė
pd(gains::PDGains, e::RodriguesVec, ė::AbstractVector) = pd(gains, SVector(e.sx, e.sy, e.sz), ė)
pd(gains::PDGains, e::Rotation{3}, ė::AbstractVector) = pd(gains, RodriguesVec(e), ė)

rotategain(gain::Number, R::Rotation) = gain
rotategain(gain::AbstractMatrix, R::Rotation) = R * gain * R'

function Spatial.transform(gains::PDGains, tf::Transform3D)
    R = rotation(tf)
    PDGains(rotategain(gains.k, R), rotategain(gains.d, R))
end

# Gains with a frame annotation
struct FramePDGains{K, D} <: AbstractPDGains
    frame::CartesianFrame3D
    gains::PDGains{K, D}
end

function pd(fgains::FramePDGains, e::FreeVector3D, ė::FreeVector3D)
    @framecheck fgains.frame e.frame
    @framecheck fgains.frame ė.frame
    FreeVector3D(fgains.frame, pd(fgains.gains, e.v, ė.v))
end

function Spatial.transform(fgains::FramePDGains, tf::Transform3D)
    @framecheck tf.from fgains.frame
    FramePDGains(tf.to, transform(fgains.gains, tf))
end

# PD control on the special Euclidean group
struct SE3PDGains{A<:Union{PDGains, FramePDGains}, L<:Union{PDGains, FramePDGains}} <: AbstractPDGains
    angular::A
    linear::L
end

function SE3PDGains(frame::CartesianFrame3D, angular::PDGains, linear::PDGains)
    SE3PDGains(FramePDGains(frame, angular), FramePDGains(frame, linear))
end

Spatial.angular(gains::SE3PDGains) = gains.angular
Spatial.linear(gains::SE3PDGains) = gains.linear

function Spatial.transform(gains::SE3PDGains, t::Transform3D)
    SE3PDGains(transform(gains.angular, t), transform(gains.linear, t))
end

struct SE3PDMethod{T} end

pd(gains::SE3PDGains, e::Transform3D, ė::Twist) = pd(gains, e, ė, SE3PDMethod{:DoubleGeodesic}())

function pd(gains::SE3PDGains, e::Transform3D, ė::Twist, ::SE3PDMethod{:DoubleGeodesic})
    # Theorem 12 in Bullo, Murray, "Proportional derivative (PD) control on the Euclidean group", 1995 (4.6 in the Technical Report).
    # Note: in Theorem 12, even though the twist and spatial acceleration are expressed in body frame, the gain Kv is expressed in base (or desired) frame,
    # since it acts on the translation part of the transform from body frame to base frame (i.e. the definition of body frame expressed in base frame),
    # and force Kv * p needs to be rotated back to body frame to match the spatial acceleration in body frame.
    # Instead, we express Kv in body frame by performing a similarity transform on Kv: R * Kv * R', where R is the rotation from actual body frame to desired body frame.
    # This turns the linear and proportional part of the PD law into R' * R * Kv * R' * p = Kv * R' * p
    # Note also that in Theorem 12, the frame in which the orientation gain Kω must be expressed is ambiguous, since R' * log(R) = log(R).
    # This explains why the Kω used here is the same as the Kω in Theorem 12.
    # Gains should be expressed in actual body frame.
    bodyframe = ė.body
    @framecheck e.from bodyframe
    @framecheck ė.base e.to
    @framecheck ė.frame bodyframe

    R = rotation(e)
    p = translation(e)
    ψ = RodriguesVec(R)

    ang = pd(angular(gains), FreeVector3D(bodyframe, ψ.sx, ψ.sy, ψ.sz), FreeVector3D(bodyframe, angular(ė)))
    lin = pd(linear(gains), FreeVector3D(bodyframe, R' * p), FreeVector3D(bodyframe, linear(ė)))
    SpatialAcceleration(ė.body, ė.base, ang, lin)
end

function pd(gains::SE3PDGains, e::Transform3D, ė::Twist, ::SE3PDMethod{:Linearized})
    bodyframe = ė.body
    @framecheck e.from bodyframe
    @framecheck ė.base e.to
    @framecheck ė.frame bodyframe

    R = rotation(e)
    p = translation(e)
    ψ = linearized_rodrigues_vec(R)

    ang = pd(angular(gains), FreeVector3D(bodyframe, ψ.sx, ψ.sy, ψ.sz), FreeVector3D(bodyframe, angular(ė)))
    lin = pd(linear(gains), FreeVector3D(bodyframe, R' * p), FreeVector3D(bodyframe, linear(ė)))
    SpatialAcceleration(ė.body, ė.base, ang, lin)
end

end # module
