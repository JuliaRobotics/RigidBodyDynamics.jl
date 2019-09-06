using RigidBodyDynamics
using Rotations
urdf = "singleBox.urdf"
mechanism = parse_urdf(urdf, floating=true) 

box = bodies(mechanism)[2]   
Nmdl = RigidBodyDynamics.Contact.hunt_crossley_hertz()   
Fmdl = RigidBodyDynamics.Contact.ViscoelasticCoulombModel(0.3, 30e3,0.3)   
scm = RigidBodyDynamics.Contact.SoftContactModel(Nmdl, Fmdl) 
for i in [-0.1, 0.1]
    for j in [-0.1, 0.1]
        for k in [-0.1, 0.1]
            point = Point3D(default_frame(box), i, j, k)
            cp = RigidBodyDynamics.Contact.ContactPoint(point, scm)
            add_contact_point!(bodies(mechanism)[2], cp)
        end
    end
end
frame = default_frame(bodies(mechanism)[1])
point = Point3D(frame, 0.0, 0.0, 0.0)
normal = FreeVector3D(frame, 0., 0., 1.)
halfspace = RigidBodyDynamics.Contact.HalfSpace3D(point, normal)

push!(mechanism.environment.halfspaces, halfspace)

function no_control!(torques::AbstractVector, t, state::MechanismState)
    for i in 1:length(torques)
        torques[i] = 0
    end
end
floating_joint = joints(mechanism)[1]
state = MechanismState(mechanism)
zero_velocity!(state)
rot = RotXYZ{Float64}(0.2, 0.4, 0.6)
quat = convert(Quat{Float64}, rot)
set_configuration!(state, floating_joint, [quat.w, quat.x, quat.y, quat.z, 0.5, 0.3, 2])

final_time = 5
ts, qs, vs = simulate(state, final_time, no_control!; Δt = 1e-6);

using MeshCatMechanisms
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));
open(mvis)
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);

