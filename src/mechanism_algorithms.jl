function subtree_mass{M}(m::Mechanism{M}, base::Tree{RigidBody{M}, Joint})
    if isroot(base)
        result = 0
    else
        result = base.vertexData.inertia.mass
    end
    for child in base.children
        result += subtree_mass(m, child)
    end
    return result
end
mass{M}(m::Mechanism{M}) = subtree_mass(m, m.tree)

function center_of_mass{C}(itr, frame::CartesianFrame3D, cache::MechanismStateCache{C})
    com = Point3D(frame, zero(Vec{3, C}))
    mass = zero(C)
    for body in itr
        if !isroot(body)
            inertia = body.inertia
            com += inertia.mass * transform(cache, Point3D(inertia.frame, inertia.centerOfMass), frame)
            mass += inertia.mass
        end
    end
    com /= mass
    return com
end

center_of_mass{M, C}(m::Mechanism{M}, cache::MechanismStateCache{C}) = center_of_mass(bodies(m), root(m).frame, cache)
