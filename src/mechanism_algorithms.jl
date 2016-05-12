function subtree_mass{T}(base::Tree{RigidBody{T}, Joint})
    result = isroot(base) ? zero(T) : base.vertexData.inertia.mass
    for child in base.children
        result += subtree_mass(child)
    end
    return result
end
mass(m::Mechanism) = subtree_mass(tree(m))
mass(state::MechanismState) = mass(state.mechanism)

function center_of_mass{X, M, C}(state::MechanismState{X, M, C}, itr)
    frame = root_body(state.mechanism).frame
    com = Point3D(frame, zero(Vec{3, C}))
    mass = zero(C)
    for body in itr
        if !isroot(body)
            inertia = body.inertia
            com += inertia.mass * transform(state, Point3D(inertia.frame, convert(Vec{3, C}, inertia.centerOfMass)), frame)
            mass += inertia.mass
        end
    end
    com /= mass
    return com
end

center_of_mass(state::MechanismState) = center_of_mass(state, bodies(state.mechanism))

function geometric_jacobian{X, M, C}(state::MechanismState{X, M, C}, path::Path{RigidBody{M}, Joint})
    flipIfNecessary = (sign::Int64, motionSubspace::GeometricJacobian) -> sign == -1 ? -motionSubspace : motionSubspace
    motionSubspaces = [flipIfNecessary(sign, motion_subspace(state, joint))::GeometricJacobian{C} for (joint, sign) in zip(path.edgeData, path.directions)]
    return hcat(motionSubspaces...)
end

kinetic_energy{X, M}(state::MechanismState{X, M}, body::RigidBody{M}) = kinetic_energy(spatial_inertia(state, body), twist_wrt_world(state, body))
function kinetic_energy{X, M}(state::MechanismState{X, M}, itr)
    return sum(body::RigidBody{M} -> kinetic_energy(state, body), itr)
end
kinetic_energy(state::MechanismState) = kinetic_energy(state, filter(b -> !isroot(b), bodies(state.mechanism)))

potential_energy{X, M, C}(state::MechanismState{X, M, C}) = -mass(state) * dot(convert(Vec{3, C}, state.mechanism.gravity), transform(state, center_of_mass(state), root_frame(state.mechanism)).v)

function mass_matrix{X, M, C}(state::MechanismState{X, M, C})
    nv = num_velocities(keys(state.motionSubspaces))
    H = zeros(C, nv, nv)

    for i = 2 : length(state.mechanism.toposortedTree)
        vi = state.mechanism.toposortedTree[i]

        # Hii
        bodyi = vi.vertexData
        jointi = vi.edgeToParentData
        vStarti = state.velocityVectorStartIndices[jointi]
        irange = vStarti : vStarti + num_velocities(jointi) - 1
        Si = motion_subspace(state, jointi)
        F = crb_inertia(state, bodyi) * Si
        H[irange, irange] = At_mul_B(Si.mat, F.mat)

        # Hji, Hij
        vj = vi.parent
        while (!isroot(vj))
            jointj = vj.edgeToParentData
            vStartj = state.velocityVectorStartIndices[jointj]
            jrange = vStartj : vStartj + num_velocities(jointj) - 1
            Sj = motion_subspace(state, jointj)
            @assert F.frame == Sj.frame
            Hji = At_mul_B(Sj.mat, F.mat)
            H[jrange, irange] = Hji
            H[irange, jrange] = Hji'
            vj = vj.parent
        end
    end
    return H
end

function momentum_matrix(state::MechanismState)
    hcat([crb_inertia(state, vertex.vertexData) * motion_subspace(state, vertex.edgeToParentData) for vertex in state.mechanism.toposortedTree[2 : end]]...)
end

function inverse_dynamics{X, M, V}(state::MechanismState{X, M}, v̇::Dict{Joint, Vector{V}}, externalWrenches::Dict{RigidBody{M}, Wrench{V}} = Dict{RigidBody{M}, Wrench{V}}())
    vertices = state.mechanism.toposortedTree
    T = promote_type(X, M, V)

    # compute spatial accelerations
    rootBody = root_body(state.mechanism)
    accels = Dict{RigidBody{M}, SpatialAcceleration{T}}(rootBody => bias_acceleration(state, rootBody))
    sizehint!(accels, length(vertices))
    for i = 2 : length(vertices)
        vertex = vertices[i]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        bias = bias_acceleration(state, body)
        S = motion_subspace(state, joint)
        @assert bias.frame == S.frame
        Sv̇ = S.mat * v̇[joint]
        joint_accel = SpatialAcceleration(bias.body, bias.base, bias.frame, bias.angular + Vec(Sv̇[1 : 3]), bias.linear + Vec(Sv̇[4 : 6]))
        accels[body] = accels[vertex.parent.vertexData] + joint_accel
    end

    # set joint wrenches equal to net wrenches
    jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
    sizehint!(jointWrenches, length(vertices) - 1)
    for i = 2 : length(vertices)
        vertex = vertices[i]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        I = spatial_inertia(state, body)
        Ṫbody = accels[body]
        Tbody = twist_wrt_world(state, body)
        wrench = newton_euler(I, Ṫbody, Tbody)
        if haskey(externalWrenches, body)
            wrench = wrench - transform(state, externalWrenches[body], wrench.frame)
        end
        jointWrenches[body] = wrench
    end

    # project joint wrench to find torques, update parent joint wrench
    ret = zeros(T, num_velocities(state.mechanism))
    for i = length(vertices) : -1 : 2
        vertex = vertices[i]
        body = vertex.vertexData
        parentBody = vertex.parent.vertexData
        joint = vertex.edgeToParentData
        jointWrench = jointWrenches[body]
        S = motion_subspace(state, joint)
        τ = joint_torque(S, jointWrench)
        vStart = state.velocityVectorStartIndices[joint]
        ret[vStart : vStart + num_velocities(joint) - 1] = τ
        if !isroot(parentBody)
            jointWrenches[parentBody] = jointWrenches[parentBody] + jointWrench # action = -reaction
        end
    end
    return ret
end
