struct SoftContactState{T, P, C}
    x::Vector{T} # minimal description of continuous state
    xsegments::Vector{SubArray{T, 1, Vector{T}, Tuple{UnitRange{Int}}, true}}
    pairs::P
    caches::C
end

function SoftContactState{T}(model::ContactModel) where {T}
    pairs = collidable_pairs(model)
    num_x = sum(pair -> num_states(pair.model), pairs)
    x = zeros(T, num_x)
    xstart = 0
    xsegments = map(pairs) do pair
        @assert pair.a.geometry isa GeometryTypes.Point || pair.b.geometry isa GeometryTypes.Point
        xend = xstart + num_states(pair.model)
        xsegment = view(x, xstart + 1 : xend) # TODO: try uview, see if it's faster
        xstart = xend
        xsegment
    end
    sorted_pairs = TypeSortedCollection(pairs)
    caches = TypeSortedCollection(
        map(pair -> collision_cache(T, pair.a.geometry, pair.b.geometry), pairs),
        TypeSortedCollections.indices(sorted_pairs)
    )
    ret = SoftContactState(x, xsegments, sorted_pairs, caches)
    reset!(ret)
    return ret
end

SoftContactState(model::ContactModel) = SoftContactState{Float64}(model)

function Base.show(io::IO, state::SoftContactState)
    print(io, "SoftContactState for ", length(state.caches), " collidable pair(s)")
end

function reset!(state::SoftContactState)
    @inbounds foreach(state.pairs, state.xsegments) do pair, xsegment
        @inbounds xsegment .= vectorize(zero_state(pair.model))
    end
    @inbounds foreach(state.caches) do cache
        reset!(cache)
    end
    return state
end

struct SoftContactResult{T}
    ẋ::Vector{T}
    ẋsegments::Vector{SubArray{T, 1, Vector{T}, Tuple{UnitRange{Int}}, true}}
    wrenches::BodyDict{Wrench{T}}
end

function SoftContactResult{T}(mechanism::Mechanism, model::ContactModel) where {T}
    pairs = collidable_pairs(model)
    num_x = sum(pair -> num_states(pair.model), pairs)
    ẋ = zeros(T, num_x)
    ẋstart = 0
    ẋsegments = map(pairs) do pair
        ẋend = ẋstart + num_states(pair.model)
        ẋsegment = view(ẋ, ẋstart + 1 : ẋend) # TODO: try uview, see if it's faster
        ẋstart = ẋend
        ẋsegment
    end
    frame = root_frame(mechanism)
    wrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, frame) for b in bodies(mechanism))
    SoftContactResult(ẋ, ẋsegments, wrenches)
end

function SoftContactResult(mechanism::Mechanism{M}, model::ContactModel) where {M}
    SoftContactResult{M}(mechanism, model)
end

function Base.show(io::IO, state::SoftContactResult)
    print(io, typeof(state), "(…)")
end

function contact_dynamics!(contact_result::SoftContactResult, contact_state::SoftContactState, mechanism_state::MechanismState)
    wrenches = contact_result.wrenches
    for bodyid in keys(wrenches)
        wrenches[bodyid] = zero(wrenches[bodyid])
    end
    # TODO: broad phase
    update_transforms!(mechanism_state)
    update_twists_wrt_world!(mechanism_state)
    frame = root_frame(mechanism_state.mechanism)
    pairs = contact_state.pairs
    xsegments = contact_state.xsegments
    ẋsegments = contact_result.ẋsegments
    caches = contact_state.caches
    handle_contact = let mechanism_state = mechanism_state, frame = frame
        function(pair, cache, xsegment, ẋsegment)
            # TODO: optimize case where a or b is root
            # TODO: make it so that CollisionElements store transforms as Transformations.
            a_to_root = transform_to_root(mechanism_state, pair.a.bodyid, false) * pair.a.transform
            b_to_root = transform_to_root(mechanism_state, pair.b.bodyid, false) * pair.b.transform
            separation, normal, closest_in_a, closest_in_b = detect_contact(cache, AffineMap(a_to_root), AffineMap(b_to_root))
            model = pair.model
            if separation < 0
                @assert isapprox(closest_in_a, closest_in_b; atol=1e-5)
                collision_location = Point3D(frame, closest_in_a)
                twist = relative_twist(mechanism_state, pair.a.bodyid, pair.b.bodyid, false)
                relative_velocity = point_velocity(twist, collision_location)
                pair_state = devectorize(model, xsegment)
                penetration = -separation
                force, pair_state_deriv = soft_contact_dynamics(model, pair_state, penetration, relative_velocity.v, normal)
                ẋsegment .= vectorize(pair_state_deriv)
                wrench_a = Wrench(collision_location, FreeVector3D(frame, force))
                wrenches[pair.a.bodyid] += wrench_a
                wrenches[pair.b.bodyid] -= wrench_a
            else
                xsegment .= vectorize(zero_state(model))
                ẋsegment .= 0
            end
            return nothing
        end
    end
    @inbounds foreach(handle_contact, pairs, caches, xsegments, ẋsegments)
    return contact_result
end
