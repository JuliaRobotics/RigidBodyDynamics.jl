module PDControl

using RigidBodyDynamics.Spatial
using StaticArrays
using Rotations
using Compat

export
    PDGains,
    DoubleGeodesicPDGains,
    pd

abstract type AbstractPDGains end

struct PDGains{K, D} <: AbstractPDGains
    k::K
    d::D
end

struct DoubleGeodesicPDGains{A<:PDGains, L<:PDGains} <: AbstractPDGains
    frame::CartesianFrame3D
    angular::A
    linear::L
end

Spatial.angular(gains::DoubleGeodesicPDGains) = gains.angular
Spatial.linear(gains::DoubleGeodesicPDGains) = gains.linear

function Spatial.transform(gains::DoubleGeodesicPDGains, t::Transform3D)
    @framecheck t.from gains.frame
    R = rotation(t)
    ang = PDGains(R * angular(gains).k * R', R * angular(gains).d * R')
    lin = PDGains(R * linear(gains).k * R', R * linear(gains).d * R')
    DoubleGeodesicPDGains(t.to, ang, lin)
end

group_error(x, xdes) = x - xdes
group_error(x::Rotation, xdes::Rotation) = inv(xdes) * x
group_error(x::Transform3D, xdes::Transform3D) = inv(xdes) * x

pd(gains::AbstractPDGains, x, xdes, ẋ, ẋdes) = pd(gains, group_error(x, xdes), -ẋdes + ẋ) # TODO: ẋ - ẋdes, but doesn't work for Twists right now

pd(gains::PDGains, e, ė) = -gains.k * e - gains.d * ė
pd(gains::PDGains, e::RodriguesVec, ė::AbstractVector) = pd(gains, SVector(e.sx, e.sy, e.sz), ė)
pd(gains::PDGains, e::Rotation{3}, ė::AbstractVector) = pd(gains, RodriguesVec(e), ė)

function pd(gains::DoubleGeodesicPDGains, e::Transform3D, ė::Twist)
    # Theorem 12 in Bullo, Murray, "Proportional derivative (PD) control on the Euclidean group", 1995 (4.6 in the Technical Report).
    # Note: in Theorem 12, even though the twist and spatial acceleration are expressed in body frame, the gain Kv is expressed in base (or desired) frame,
    # since it acts on the translation part of the transform from body frame to base frame (i.e. the definition of body frame expressed in base frame),
    # and force Kv * p needs to be rotated back to body frame to match the spatial acceleration in body frame.
    # Instead, we express Kv in body frame by performing a similarity transform on Kv: R * Kv * R', where R is the rotation from actual body frame to desired body frame.
    # This turns the linear and proportional part of the PD law into R' * R * Kv * R' * p = Kv * R' * p
    # Note also that in Theorem 12, the frame in which the orientation gain Kω must be expressed is ambiguous, since R' * log(R) = log(R).
    # This explains why the Kω used here is the same as the Kω in Theorem 12.
    @framecheck ė.body e.from
    @framecheck ė.base e.to
    @framecheck ė.frame ė.body
    @framecheck gains.frame ė.body # gains should be expressed in actual body frame
    R = rotation(e)
    p = translation(e)
    SpatialAcceleration(ė.body, ė.base, ė.frame, pd(angular(gains), R, ė.angular), pd(linear(gains), R' * p, ė.linear))
end

end # module
