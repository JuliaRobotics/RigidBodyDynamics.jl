@indextype CollisionGeometryID

struct CollisionGeometryMount{T, G}
    bodyid::BodyID
    transform::T # to body's default frame
    geometry::G
end

mutable struct CollidablePair{A<:CollisionGeometryMount, B<:CollisionGeometryMount, M<:ContactForceModel}
    a::A
    b::B
    model::M
end

const CollisionGroup = Vector{CollisionGeometryMount}

struct ContactModel
    collision_groups::Vector{CollisionGroup}
    contact_force_models::Dict{UnorderedPair{CollisionGroup}, ContactForceModel}
end

function ContactModel()
    collision_groups = CollisionGroup[]
    contact_force_models = Dict{UnorderedPair{CollisionGroup}, ContactForceModel}()
    ContactModel(collision_groups, contact_force_models)
end

function add_collision_group!(model::ContactModel, group::CollisionGroup)
    push!(model.collision_groups, group)
    return nothing
end

function set_contact_force_model!(model::ContactModel, group_i::CollisionGroup, group_j::CollisionGroup, force_model::ContactForceModel)
    model.contact_force_models[UnorderedPair(group_i, group_j)] = force_model
end

function collidable_pairs(contact_model::ContactModel)
    ret = CollidablePair[]
    collision_groups = contact_model.collision_groups
    for i in 1 : length(collision_groups)
        group_i = collision_groups[i]
        for j in i : length(collision_groups)
            group_j = collision_groups[j]
            contact_force_model = contact_model.contact_force_models[UnorderedPair(group_i, group_j)]
            for k in 1 : length(group_i)
                lstart = group_i == group_j ? k : 1
                a = group_i[k]
                for l in lstart : length(group_j)
                    b = group_j[l]
                    push!(ret, CollidablePair(a, b, contact_force_model))
                end
            end
        end
    end
    return ret
end
