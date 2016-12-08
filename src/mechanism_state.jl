# State information pertaining to a single joint its successor body.
immutable StateElement{J<:Joint, R<:RigidBody, X<:Real, C<:Real, N, L}
    joint::J
    body::R
    q::VectorSegment{X}
    v::VectorSegment{X}
    parent::Int64
    children::Vector{Int64}
    beforeJointToParent::Transform3D{C}

    transformToWorld::CacheElement{Transform3D{C}}
    twist::CacheElement{Twist{C}}
    biasAcceleration::CacheElement{SpatialAcceleration{C}}
    motionSubspace::CacheElement{GeometricJacobian{SMatrix{3, N, C, L}}}
    inertia::CacheElement{SpatialInertia{C}}
    crbInertia::CacheElement{SpatialInertia{C}}

    function StateElement(
            joint::J, body::R,
            q::VectorSegment{X}, v::VectorSegment{X},
            parent::Int64, children::Vector{Int64},
            beforeJointToParent::Transform3D{C})
        transformToWorld = CacheElement{Transform3D{C}}()
        twist = CacheElement{Twist{C}}()
        biasAcceleration = CacheElement{SpatialAcceleration{C}}()
        motionSubspace = CacheElement{GeometricJacobian{SMatrix{3, N, C, L}}}()
        inertia = CacheElement{SpatialInertia{C}}()
        crbInertia = CacheElement{SpatialInertia{C}}()
        new(joint, body, q, v, parent, children, beforeJointToParent, transformToWorld, twist, biasAcceleration, motionSubspace, inertia, crbInertia)
    end
end

function StateElement{R<:RigidBody, J<:Joint, X<:Real}(
        joint::J, body::R,
        q::VectorSegment{X}, v::VectorSegment{X},
        parent::Int64, children::Vector{Int64},
        beforeJointToParent::Transform3D)
    C = promote_type(eltype(R), eltype(J), X)
    N = num_velocities(joint)
    L = 3 * N
    StateElement{J, R, X, C, N, L}(joint, body, q, v, parent, children, convert(Transform3D{C}, beforeJointToParent))
end

configuration(element::StateElement) = element.q
velocity(element::StateElement) = state.v
configuration_range(element::StateElement) = first(parentindexes(element.q))
velocity_range(element::StateElement) = first(parentindexes(element.v))
parent_frame(element::StateElement) = element.beforeJointToParent.to
zero_configuration!(element::StateElement) = (zero_configuration!(element.joint, element.q))
rand_configuration!(element::StateElement) = (rand_configuration!(element.joint, element.q))

function setdirty!(element::StateElement)
    setdirty!(element.transformToWorld)
    setdirty!(element.twist)
    setdirty!(element.biasAcceleration)
    setdirty!(element.motionSubspace)
    setdirty!(element.inertia)
    setdirty!(element.crbInertia)
end

# State of an entire Mechanism
# The state information pertaining to rigid bodies in toposortedStateVertices is
# currently stored in the Mechanism's root frame.
immutable MechanismState{X<:Real, E}
    mechanism::Mechanism
    q::Vector{X}
    v::Vector{X}
    elements::E
end

function MechanismState{X, M}(t::Type{X}, mechanism::Mechanism{M})
    q = Vector{X}(num_positions(mechanism))
    v = zeros(X, num_velocities(mechanism))

    elements = Vector{StateElement}()
    qStart, vStart = 1, 1
    nonRootVertices = collect(filter(v -> !isroot(v), toposort(tree(mechanism))))
    for vertex in nonRootVertices
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        parentIndex = findfirst(e -> e.body == vertex_data(parent(vertex)), elements)
        # children = find(v -> v ∈ children(vertex), nonRootVertices)
        childIndices = Vector{Int64}() # TODO: clean up
        for (i, vertex2) in enumerate(nonRootVertices)
            if parent(vertex2) === vertex
                push!(childIndices, i)
            end
        end
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        beforeJointToParent = mechanism.jointToJointTransforms[joint]
        qJoint = view(q, qStart : qEnd)
        vJoint = view(v, vStart : vEnd)
        zero_configuration!(joint, qJoint)
        push!(elements, StateElement(joint, body, qJoint, vJoint, parentIndex, childIndices, beforeJointToParent))
        qStart, vStart = qEnd + 1, vEnd + 1
    end
    MechanismState(mechanism, q, v, tuple(elements...))
end

show{X}(io::IO, ::MechanismState{X}) = print(io, "MechanismState{$X, …}(…)")
num_positions(state::MechanismState) = length(state.q)
num_velocities(state::MechanismState) = length(state.v)
eltype{X}(state::MechanismState{X}) = X
state_element(state::MechanismState, body::RigidBody) = state.elements[findfirst(e -> e.body == body, state.elements)] # FIXME: linear time
state_element(state::MechanismState, joint::Joint) = state.elements[findfirst(e -> e.joint == joint, state.elements)] # FIXME: linear time
configuration(state::MechanismState, joint::Joint) = configuration(edge_to_parent_data(state_element(state, joint)))
velocity(state::MechanismState, joint::Joint) = velocity(edge_to_parent_data(state_element(state, joint)))
setdirty!(state::MechanismState) = (for element in state.elements setdirty!(element) end)
zero_configuration!(state::MechanismState) = (for element in state.elements zero_configuration!(element) end; setdirty!(state))
zero_velocity!(state::MechanismState) = (fill!(state.v,  zero(eltype(state.v))); setdirty!(state))
zero!(state::MechanismState) = (zero_configuration!(state); zero_velocity!(state))
rand_configuration!(state::MechanismState) = (for element in state.elements rand_configuration!(element) end; setdirty!(state))
rand_velocity!(state::MechanismState) = (rand!(state.v); setdirty!(state))
rand!(state::MechanismState) = (rand_configuration!(state); rand_velocity!(state))
configuration_vector(state::MechanismState) = state.q
velocity_vector(state::MechanismState) = state.v
state_vector(state::MechanismState) = [configuration_vector(state); velocity_vector(state)] # TODO: consider having q and v be views into x
configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) = vcat([configuration(state, joint) for joint in path.edgeData]...)
velocity_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) = vcat([velocity(state, joint) for joint in path.edgeData]...)

function set_configuration!(state::MechanismState, joint::Joint, q::AbstractVector)
    configuration(state, joint)[:] = q
    setdirty!(state)
end

function set_velocity!(state::MechanismState, joint::Joint, v::AbstractVector)
    velocity(state, joint)[:] = v
    setdirty!(state)
end

function set_configuration!(state::MechanismState, q::AbstractVector)
    copy!(state.q, q)
    setdirty!(state)
end

function set_velocity!(state::MechanismState, v::AbstractVector)
    copy!(state.v, v)
    setdirty!(state)
end

function set!(state::MechanismState, x::AbstractVector)
    nq = num_positions(state)
    nv = num_velocities(state)
    length(x) == nq + nv || error("wrong size")
    @inbounds copy!(state.q, 1, x, 1, nq)
    @inbounds copy!(state.v, 1, x, nq + 1, nv)
    setdirty!(state)
end

# the following functions return quantities expressed in world frame and w.r.t. world frame (where applicable)
function transform_to_root(state::MechanismState, element::StateElement)
    @cache_element_get!(element.transformToWorld, begin
        jointTransform = element.beforeJointToParent * joint_transform(element.joint, element.q)
        ret = element.parent == 0 ? jointTransform : transform_to_root(state, state.elements[element.parent]) * jointTransform
    end)
end

function twist_wrt_world(state::MechanismState, element::StateElement)
    @cache_element_get!(element.twist, begin
        jointTwist = change_base(joint_twist(element.joint, element.q, element.v), parent_frame(element))
        jointTwist = transform(jointTwist, transform_to_root(state, element))
        ret = element.parent == 0 ? change_base(jointTwist, root_frame(state.mechanism)) : twist_wrt_world(state, state.elements[element.parent]) + jointTwist
    end)
end

function bias_acceleration(state::MechanismState, element::StateElement)
    @cache_element_get!(element.biasAcceleration, begin
        toRoot = transform_to_root(state, element)
        jointBias = bias_acceleration(element.joint, element.q, element.v)
        twistWrtWorld = transform(twist_wrt_world(state, element), inv(toRoot)) # TODO
        jointTwist = change_base(joint_twist(element.joint, element.q, element.v), parent_frame(element)) # TODO
        jointBias = transform(jointBias, toRoot, twistWrtWorld, jointTwist)
        ret = element.parent == 0 ? change_base(jointBias, root_frame(state.mechanism)) : bias_acceleration(state, state.elements[element.parent]) + jointBias
    end)
end

function motion_subspace(state::MechanismState, element::StateElement)
    @cache_element_get!(element.motionSubspace,
        transform(motion_subspace(element.joint, element.q), transform_to_root(state, element)))
end

function spatial_inertia(state::MechanismState, element::StateElement)
    @cache_element_get!(element.inertia,
        transform(spatial_inertia(element.body), transform_to_root(state, element)))
end

function crb_inertia(state::MechanismState, element::StateElement)
    @cache_element_get!(element.crbInertia, begin
        ret = spatial_inertia(state, element)
        for child in element.children
            ret += crb_inertia(state, state.elements[child])
        end
        ret
    end)
end

function newton_euler(state::MechanismState, element::StateElement, accel::SpatialAcceleration)
    inertia = spatial_inertia(state, element)
    twist = twist_wrt_world(state, element)
    newton_euler(inertia, accel, twist)
end

function momentum(state::MechanismState, element::StateElement)
    inertia = spatial_inertia(state, element)
    twist = twist_wrt_world(state, element)
    inertia * twist
end
momentum_rate_bias(state::MechanismState, element::StateElement) = newton_euler(state, element, bias_acceleration(state, element))
kinetic_energy(state::MechanismState, element::StateElement) = kinetic_energy(spatial_inertia(state, element), twist_wrt_world(state, element))

function configuration_derivative!(out::StridedVector, state::MechanismState)
    for element in state.elements
        q = configuration(element)
        v = velocity(element)
        q̇ = UnsafeVectorView(out, configuration_range(element))
        velocity_to_configuration_derivative!(element.joint, q̇, q, v)
    end
end

function configuration_derivative{X}(state::MechanismState{X})
    ret = Vector{X}(num_positions(state))
    configuration_derivative!(ret, state)
    ret
end

function transform_to_root(state::MechanismState, frame::CartesianFrame3D)
    body = state.mechanism.bodyFixedFrameToBody[frame]
    tf = transform_to_root(state, state_element(state, body))
    if tf.from != frame
        tf = tf * find_body_fixed_frame_definition(state.mechanism, body, frame) # TODO: consider caching
    end
    tf
end

motion_subspace(state::MechanismState, joint::Joint) = motion_subspace(state, state_element(state, joint))

for fun in (:twist_wrt_world, :bias_acceleration, :spatial_inertia, :crb_inertia, :momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun(state::MechanismState, body::RigidBody) = $fun(state, state_element(state, body))
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy)
    @eval function $fun(state::MechanismState)
        ret = $fun(state, state.elements[1])
        for i = 2 : length(state.elements)
            ret += $fun(state, state.elements[i])
        end
        ret
    end
    @eval $fun(state::MechanismState, body_itr) = sum($fun(state, body) for body in body_itr)
end

function relative_transform(state::MechanismState, from::CartesianFrame3D, to::CartesianFrame3D)
    rootFrame = root_frame(state.mechanism)
    if to == rootFrame
        return transform_to_root(state, from)
    elseif from == rootFrame
        return inv(transform_to_root(state, to))
    else
        return inv(transform_to_root(state, to)) * transform_to_root(state, from)
    end
end

function relative_twist(state::MechanismState, body::RigidBody, base::RigidBody)
    rootBody = root_body(state.mechanism)
    if base == rootBody
        return twist_wrt_world(state, body)
    elseif body == rootBody
        return -twist_wrt_world(state, base)
    else
        return -twist_wrt_world(state, base) + twist_wrt_world(state, body)
    end
 end

function relative_twist(state::MechanismState, bodyFrame::CartesianFrame3D, baseFrame::CartesianFrame3D)
    twist = relative_twist(state, state.mechanism.bodyFixedFrameToBody[bodyFrame], state.mechanism.bodyFixedFrameToBody[baseFrame])
    Twist(bodyFrame, baseFrame, twist.frame, twist.angular, twist.linear)
end

for VectorType in (:Point3D, :FreeVector3D, :Twist, :Momentum, :Wrench)
    @eval begin
        function transform(state::MechanismState, v::$VectorType, to::CartesianFrame3D)::similar_type(typeof(v), promote_type(cache_eltype(state), eltype(v)))
            # TODO: consider transforming in steps, so that computing the relative transform is not necessary
            v.frame == to ? v : transform(v, relative_transform(state, v.frame, to))
        end
    end
end

function transform(state::MechanismState, accel::SpatialAcceleration, to::CartesianFrame3D)
    accel.frame == to && return accel # nothing to be done
    oldToRoot = transform_to_root(state, accel.frame)
    rootToOld = inv(oldToRoot)
    twistOfBodyWrtBase = transform(relative_twist(state, accel.body, accel.base), rootToOld)
    twistOfOldWrtNew = transform(relative_twist(state, accel.frame, to), rootToOld)
    oldToNew = inv(transform_to_root(state, to)) * oldToRoot
    transform(accel, oldToNew, twistOfOldWrtNew, twistOfBodyWrtBase)
end

function local_coordinates!(state::MechanismState, ϕ::StridedVector, ϕd::StridedVector, q0::StridedVector)
    mechanism = state.mechanism
    for element in state.elements
        qRange = configuration_range(element)
        vRange = velocity_range(element)
        ϕjoint = UnsafeVectorView(ϕ, vRange)
        ϕdjoint = UnsafeVectorView(ϕd, vRange)
        q0joint = UnsafeVectorView(q0, qRange)
        qjoint = configuration(element)
        vjoint = velocity(element)
        local_coordinates!(element.joint, ϕjoint, ϕdjoint, q0joint, qjoint, vjoint)
    end
end

function global_coordinates!(state::MechanismState, q0::StridedVector, ϕ::StridedVector)
    mechanism = state.mechanism
    for vertex in state.elements
        q0joint = UnsafeVectorView(q0, configuration_range(element))
        ϕjoint = UnsafeVectorView(ϕ, velocity_range(element))
        qjoint = configuration(element)
        global_coordinates!(element.joint, qjoint, q0joint, ϕjoint)
    end
end
