const CollisionGroup = Vector{CollisionElement}

struct ContactModel
    collision_groups::Vector{CollisionGroup}
    contact_force_models::Dict{UnorderedPair{CollisionGroup}, ContactForceModel}
end

function ContactModel()
    collision_groups = CollisionGroup[]
    contact_force_models = Dict{UnorderedPair{CollisionGroup}, ContactForceModel}()
    ContactModel(collision_groups, contact_force_models)
end

function Base.show(io::IO, model::ContactModel)
    print(io, "ContactModel with ", length(model.collision_groups), " collision group(s)")
end

function Base.push!(model::ContactModel, group::CollisionGroup)
    push!(model.collision_groups, group)
    return model
end

function set_contact_force_model!(
        model::ContactModel,
        group_i::CollisionGroup,
        group_j::CollisionGroup,
        force_model::ContactForceModel)
    model.contact_force_models[UnorderedPair(group_i, group_j)] = force_model
end

function collidable_pairs(model::ContactModel)
    ret = CollidablePair[]
    collision_groups = model.collision_groups
    contact_force_models = model.contact_force_models
    for i in 1 : length(collision_groups)
        group_i = collision_groups[i]
        for j in i : length(collision_groups)
            group_j = collision_groups[j]
            unordered_pair = UnorderedPair(group_i, group_j)
            if haskey(contact_force_models, unordered_pair)
                contact_force_model = model.contact_force_models[unordered_pair]
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
    end
    return ret
end

function num_continuous_states(model::ContactModel)
    # TODO: make more efficient
    pairs = collidable_pairs(model)
    sum(pair -> num_states(pair.model), pairs)
end
