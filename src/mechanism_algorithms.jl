function subtree_mass{T}(base::Tree{RigidBody{T}, Joint})
    if isroot(base)
        result = 0
    else
        result = base.vertexData.inertia.mass
    end
    for child in base.children
        result += subtree_mass(child)
    end
    return result
end
mass(m::Mechanism) = subtree_mass(tree(m))
mass(cache::MechanismStateCache) = mass(cache.mechanism)

function center_of_mass{C}(cache::MechanismStateCache{C}, itr)
    frame = root_body(cache.mechanism).frame
    com = Point3D(frame, zero(Vec{3, C}))
    mass = zero(C)
    for body in itr
        if !isroot(body)
            inertia = body.inertia
            com += inertia.mass * transform(cache, Point3D(inertia.frame, convert(Vec{3, C}, inertia.centerOfMass)), frame)
            mass += inertia.mass
        end
    end
    com /= mass
    return com
end

center_of_mass(cache::MechanismStateCache) = center_of_mass(cache, bodies(cache.mechanism))

function geometric_jacobian{C, M}(cache::MechanismStateCache{C}, path::Path{RigidBody{M}, Joint})
    flipIfNecessary = (sign::Int64, motionSubspace::GeometricJacobian{C}) -> sign == -1 ? -motionSubspace : motionSubspace
    motionSubspaces = [flipIfNecessary(sign, motion_subspace(cache, joint))::GeometricJacobian{C} for (joint, sign) in zip(path.edgeData, path.directions)]
    return hcat(motionSubspaces...)
end

kinetic_energy{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}) = kinetic_energy(spatial_inertia(cache, body), twist_wrt_world(cache, body))
function kinetic_energy{C, M}(cache::MechanismStateCache{C, M}, itr)
    return sum(body::RigidBody{M} -> kinetic_energy(cache, body), itr)
end
kinetic_energy(cache::MechanismStateCache) = kinetic_energy(cache, filter(b -> !isroot(b), bodies(cache.mechanism)))

potential_energy{C}(cache::MechanismStateCache{C}) = -mass(cache) * dot(convert(Vec{3, C}, cache.mechanism.gravity), transform(cache, center_of_mass(cache), root_frame(cache.mechanism)).v)

function mass_matrix{C}(cache::MechanismStateCache{C})
    nv = num_velocities(keys(cache.motionSubspaces))
    H = zeros(C, nv, nv)

    for i = 2 : length(cache.mechanism.toposortedTree)
        vi = cache.mechanism.toposortedTree[i]

        # Hii
        bodyi = vi.vertexData
        jointi = vi.edgeToParentData
        vStarti = cache.velocityVectorStartIndices[jointi]
        irange = vStarti : vStarti + num_velocities(jointi) - 1
        Si = motion_subspace(cache, jointi)
        F = crb_inertia(cache, bodyi) * Si
        H[irange, irange] = At_mul_B(Si.mat, F.mat)

        # Hji, Hij
        vj = vi.parent
        while (!isroot(vj))
            jointj = vj.edgeToParentData
            vStartj = cache.velocityVectorStartIndices[jointj]
            jrange = vStartj : vStartj + num_velocities(jointj) - 1
            Sj = motion_subspace(cache, jointj)
            @assert F.frame == Sj.frame
            Hji = At_mul_B(Sj.mat, F.mat)
            H[jrange, irange] = Hji
            H[irange, jrange] = Hji'
            vj = vj.parent
        end
    end
    return H
end

function momentum_matrix(cache::MechanismStateCache)
    hcat([crb_inertia(cache, vertex.vertexData) * motion_subspace(cache, vertex.edgeToParentData) for vertex in cache.mechanism.toposortedTree[2 : end]]...)
end

function inverse_dynamics{C, M, V}(cache::MechanismStateCache{C, M}, v̇::Dict{Joint, Vector{V}}, externalWrenches::Dict{RigidBody{M}, Wrench{V}} = Dict{RigidBody{M}, Wrench{V}}())
    vertices = cache.mechanism.toposortedTree
    T = promote_type(C, V)

    # compute spatial accelerations
    rootBody = root_body(cache.mechanism)
    accels = Dict{RigidBody{M}, SpatialAcceleration{T}}(rootBody => SpatialAcceleration(rootBody.frame, rootBody.frame, rootBody.frame, zero(Vec{3, T}), convert(Vec{3, T}, cache.mechanism.gravity)))
    sizehint!(accels, length(vertices))
    for i = 2 : length(vertices)
        vertex = vertices[i]
        body = vertex.vertexData
        joint = vertex.edgeToParentData

        bias = bias_acceleration(cache, body)
        S = motion_subspace(cache, joint)
        @assert bias.frame == S.frame

        Sv̇ = S.mat * v̇[joint]
        angular = bias.angular + Vec(Sv̇[1 : 3])
        linear = bias.linear + Vec(Sv̇[4 : 6])

        parentAccel = accels[vertex.parent.vertexData]
        angular += parentAccel.angular
        linear += parentAccel.linear
        accels[body] = SpatialAcceleration(bias.body, bias.base, bias.frame, angular, linear)
    end

    # initialize joint wrenches = net wrenches
    jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
    sizehint!(jointWrenches, length(vertices) - 1)
    for i = 2 : length(vertices)
        vertex = vertices[i]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        I = spatial_inertia(cache, body)
        Ṫbody = accels[body]
        Tbody = twist_wrt_world(cache, body)
        wrench = newton_euler(I, Ṫbody, Tbody)
        if haskey(externalWrenches, body)
            wrench = wrench - transform(cache, externalWrenches[body], wrench.frame)
        end
        jointWrenches[body] = wrench
    end

    # project joint wrench to find torques, update parent joint wrench
    ret = zeros(T, num_velocities(cache.mechanism))
    for i = length(vertices) : -1 : 2
        vertex = vertices[i]
        body = vertex.vertexData
        parentBody = vertex.parent.vertexData
        joint = vertex.edgeToParentData
        jointWrench = jointWrenches[body]
        S = motion_subspace(cache, joint)
        τ = joint_torque(S, jointWrench)
        vStart = cache.velocityVectorStartIndices[joint]
        ret[vStart : vStart + num_velocities(joint) - 1] = τ
        if !isroot(parentBody)
            jointWrenches[parentBody] = jointWrenches[parentBody] + jointWrench
        end
    end
    return ret
end
