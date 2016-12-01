# State information pertaining to a single joint
immutable JointState{X<:Real, M<:Real, C<:Real}
    joint::Joint{M}
    q::VectorSegment{X}
    v::VectorSegment{X}
    beforeJointToParent::Transform3D{C}
    afterJointToParent::CacheElement{Transform3D{C}}
    twist::CacheElement{Twist{C}}
    biasAcceleration::CacheElement{SpatialAcceleration{C}}
    motionSubspace::CacheElement{MotionSubspace{C}}

    function JointState(joint::Joint{M}, beforeJointToParent::Transform3D{C}, q::VectorSegment{X}, v::VectorSegment{X})
        afterJointToParent = CacheElement{Transform3D{C}}()
        twist = CacheElement{Twist{C}}()
        biasAcceleration = CacheElement{SpatialAcceleration{C}}()
        motionSubspace = CacheElement{MotionSubspace{C}}()
        new(joint, q, v, beforeJointToParent, afterJointToParent, twist, biasAcceleration, motionSubspace)
    end
end

function JointState{X, M}(joint::Joint{M}, beforeJointToParent::Transform3D{M}, q::VectorSegment{X}, v::VectorSegment{X})
    C = promote_type(M, X)
    JointState{X, M, C}(joint, convert(Transform3D{C}, beforeJointToParent), q, v)
end

configuration(state::JointState) = state.q
velocity(state::JointState) = state.v
configuration_range(state::JointState) = first(parentindexes(state.q))
velocity_range(state::JointState) = first(parentindexes(state.v))
parent_frame(state::JointState) = state.beforeJointToParent.to
transform(state::JointState) = @cache_element_get!(state.afterJointToParent, state.beforeJointToParent * joint_transform(state.joint, state.q))
twist(state::JointState) = @cache_element_get!(state.twist, change_base(joint_twist(state.joint, state.q, state.v), parent_frame(state)))
bias_acceleration(state::JointState) = @cache_element_get!(state.biasAcceleration, change_base(bias_acceleration(state.joint, state.q, state.v), parent_frame(state)))
motion_subspace(state::JointState) = @cache_element_get!(state.motionSubspace, change_base(motion_subspace(state.joint, state.q), parent_frame(state)))

zero_configuration!(state::JointState) = (zero_configuration!(state.joint, state.q))
rand_configuration!(state::JointState) = (rand_configuration!(state.joint, state.q))

function setdirty!(state::JointState)
    setdirty!(state.afterJointToParent)
    setdirty!(state.twist)
    setdirty!(state.biasAcceleration)
    setdirty!(state.motionSubspace)
end

# State information pertaining to a single rigid body
immutable RigidBodyState{M<:Real, C<:Real}
    body::RigidBody{M}
    transformToWorld::CacheElement{Transform3D{C}}
    twist::CacheElement{Twist{C}}
    biasAcceleration::CacheElement{SpatialAcceleration{C}}
    motionSubspace::CacheElement{MotionSubspace{C}} # TODO: should this be here?
    inertia::CacheElement{SpatialInertia{C}}
    crbInertia::CacheElement{SpatialInertia{C}}

    function RigidBodyState(body::RigidBody{M}, isroot::Bool)
        transformToWorld = CacheElement{Transform3D{C}}()
        twist = CacheElement{Twist{C}}()
        biasAcceleration = CacheElement{SpatialAcceleration{C}}()
        motionSubspace = CacheElement{MotionSubspace{C}}()
        inertia = CacheElement{SpatialInertia{C}}()
        crbInertia = CacheElement{SpatialInertia{C}}()

        if isroot
            update!(transformToWorld, Transform3D(C, body.frame))
            update!(twist, zero(Twist{C}, body.frame, body.frame, body.frame))
            update!(biasAcceleration, zero(SpatialAcceleration{C}, body.frame, body.frame, body.frame))
        end

        new(body, transformToWorld, twist, biasAcceleration, motionSubspace, inertia, crbInertia)
    end
end

RigidBodyState{M, X}(body::RigidBody{M}, ::Type{X}, isroot::Bool) = RigidBodyState{M, promote_type(M, X)}(body, isroot)

function setdirty!(state::RigidBodyState)
    setdirty!(state.transformToWorld)
    setdirty!(state.twist)
    setdirty!(state.biasAcceleration)
    setdirty!(state.motionSubspace)
    setdirty!(state.inertia)
    setdirty!(state.crbInertia)
end

# State of an entire Mechanism
# The state information pertaining to rigid bodies in toposortedStateVertices is
# currently stored in the Mechanism's root frame.
immutable MechanismState{X<:Real, M<:Real, C<:Real}
    mechanism::Mechanism{M}
    q::Vector{X}
    v::Vector{X}
    toposortedStateVertices::Vector{TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}}
    nonRootTopoSortedStateVertices::VectorSegment{TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}} # because of https://github.com/JuliaLang/julia/issues/14955

    function MechanismState(::Type{X}, mechanism::Mechanism{M})
        q = Vector{X}(num_positions(mechanism))
        v = Vector{X}(num_velocities(mechanism))
        rootBodyState = RigidBodyState(root_body(mechanism), X, true)
        tree = Tree{RigidBodyState{M, C}, JointState{X, M, C}}(rootBodyState)
        jointStates = Dict{Joint{M}, JointState{X, M, C}}()
        bodyStates = Dict{RigidBody{M}, RigidBodyState{M, C}}()
        for vertex in filter(x -> !isroot(x), mechanism.toposortedTree)
            body = vertex_data(vertex)
            bodyState = RigidBodyState(body, X, false)
            joint = edge_to_parent_data(vertex)
            parentBody = vertex_data(parent(vertex))
            parentStateVertex = findfirst(v -> vertex_data(v).body == parentBody, tree)
            qJoint = view(q, mechanism.qRanges[joint])
            vJoint = view(v, mechanism.vRanges[joint])
            beforeJointToParent = mechanism.jointToJointTransforms[joint]
            jointState = JointState(joint, beforeJointToParent, qJoint, vJoint)
            insert!(parentStateVertex, bodyState, jointState)
        end
        vertices = toposort(tree)
        new(mechanism, q, v, vertices, view(vertices, 2 : length(vertices)))
    end
end
MechanismState{X, M}(t::Type{X}, mechanism::Mechanism{M}) = MechanismState{X, M, promote_type(X, M)}(t, mechanism)

show{X, M, C}(io::IO, ::MechanismState{X, M, C}) = print(io, "MechanismState{$X, $M, $C}(…)")
num_positions(state::MechanismState) = length(state.q)
num_velocities(state::MechanismState) = length(state.v)
state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C
state_vertex(state::MechanismState, body::RigidBody) = state.toposortedStateVertices[findfirst(v -> vertex_data(v).body == body, state.toposortedStateVertices)] # FIXME: linear time
state_vertex(state::MechanismState, joint::Joint) = state.toposortedStateVertices[findfirst(v -> !isroot(v) && edge_to_parent_data(v).joint == joint, state.toposortedStateVertices)] # FIXME: linear time
configuration(state::MechanismState, joint::Joint) = edge_to_parent_data(state_vertex(state, joint)).q
velocity(state::MechanismState, joint::Joint) = edge_to_parent_data(state_vertex(state, joint)).v
non_root_vertices(state::MechanismState) = state.nonRootTopoSortedStateVertices

function setdirty!(state::MechanismState)
    for vertex in non_root_vertices(state)
        setdirty!(vertex_data(vertex))
        setdirty!(edge_to_parent_data(vertex))
    end
end

function zero_configuration!(state::MechanismState)
    for vertex in non_root_vertices(state)
        zero_configuration!(edge_to_parent_data(vertex))
    end
    setdirty!(state)
end

function zero_velocity!(state::MechanismState)
    X = eltype(state.v)
    fill!(state.v,  zero(X))
    setdirty!(state)
end

zero!(state::MechanismState) = begin zero_configuration!(state); zero_velocity!(state) end

function rand_configuration!(state::MechanismState)
    for vertex in non_root_vertices(state)
        rand_configuration!(edge_to_parent_data(vertex))
    end
    setdirty!(state)
end

function rand_velocity!(state::MechanismState)
    rand!(state.v)
    setdirty!(state)
end

rand!(state::MechanismState) = begin rand_configuration!(state); rand_velocity!(state) end

configuration_vector(state::MechanismState) = state.q
velocity_vector(state::MechanismState) = state.v
state_vector(state::MechanismState) = [configuration_vector(state); velocity_vector(state)]

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
function transform_to_root{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).transformToWorld,
        transform_to_root(parent(vertex)) * transform(edge_to_parent_data(vertex)))
end

function twist_wrt_world{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).twist,
        twist_wrt_world(parent(vertex)) + transform(twist(edge_to_parent_data(vertex)), transform_to_root(vertex)))
end

function bias_acceleration{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).biasAcceleration, begin
        parentVertex = parent(vertex)
        parentBias = bias_acceleration(parentVertex)
        toRoot = transform_to_root(vertex)
        jointBias = bias_acceleration(edge_to_parent_data(vertex))
        twistWrtWorld = transform(twist_wrt_world(vertex), inv(toRoot)) # TODO
        jointTwist = twist(edge_to_parent_data(vertex))
        jointBias = transform(jointBias, toRoot, twistWrtWorld, jointTwist)
        parentBias + jointBias
    end)
end

function motion_subspace{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).motionSubspace,
        transform(motion_subspace(edge_to_parent_data(vertex)), transform_to_root(vertex)))
end

function spatial_inertia{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).inertia,
        transform(spatial_inertia(vertex_data(vertex).body), transform_to_root(vertex)))
end

function crb_inertia{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).crbInertia, begin
        ret = spatial_inertia(vertex)
        for child in children(vertex)
            ret += crb_inertia(child)
        end
        ret
    end)
end

function newton_euler{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}, accel::SpatialAcceleration)
    inertia = spatial_inertia(vertex)
    twist = twist_wrt_world(vertex)
    newton_euler(inertia, accel, twist)
end

momentum{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = spatial_inertia(vertex) * twist_wrt_world(vertex)
momentum_rate_bias{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = newton_euler(vertex, bias_acceleration(vertex))
kinetic_energy{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = kinetic_energy(spatial_inertia(vertex), twist_wrt_world(vertex))

function configuration_derivative!{X}(out::AbstractVector{X}, state::MechanismState{X})
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        q = configuration(jointState)
        v = velocity(jointState)
        @inbounds q̇ = view(out, configuration_range(jointState)) # TODO: allocates
        velocity_to_configuration_derivative!(jointState.joint, q̇, q, v)
    end
end

function configuration_derivative{X}(state::MechanismState{X})
    ret = Vector{X}(num_positions(state.mechanism))
    configuration_derivative!(ret, state)
    ret
end

function transform_to_root(state::MechanismState, frame::CartesianFrame3D)
    body = state.mechanism.bodyFixedFrameToBody[frame]
    transform = transform_to_root(state_vertex(state, body))
    if transform.from != frame
        transform = transform * find_body_fixed_frame_definition(state.mechanism, body, frame) # TODO: consider caching
    end
    transform
end

motion_subspace(state::MechanismState, joint::Joint) = motion_subspace(state_vertex(state, joint))

for fun in (:twist_wrt_world, :bias_acceleration, :spatial_inertia, :crb_inertia, :momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}, body::RigidBody{M}) = $fun(state_vertex(state, body))
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}) = sum($fun, non_root_vertices(state))
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}, body_itr) = sum($fun(state, body) for body in body_itr)
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

function local_coordinates!(state::MechanismState, ϕ::AbstractVector, ϕ̇::AbstractVector, q0::AbstractVector)
    mechanism = state.mechanism
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        qRange = configuration_range(jointState)
        vRange = velocity_range(jointState)
        @inbounds ϕjoint = view(ϕ, vRange) # TODO: allocates
        @inbounds ϕ̇joint = view(ϕ̇, vRange) # TODO: allocates
        @inbounds q0joint = view(q0, qRange) # TODO: allocates
        qjoint = configuration(jointState)
        vjoint = velocity(jointState)
        local_coordinates!(jointState.joint, ϕjoint, ϕ̇joint, q0joint, qjoint, vjoint)
    end
end

function global_coordinates!(state::MechanismState, q0::AbstractVector, ϕ::AbstractVector)
    mechanism = state.mechanism
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        @inbounds q0joint = view(q0, configuration_range(jointState)) # TODO: allocates
        @inbounds ϕjoint = view(ϕ, velocity_range(jointState)) # TODO: allocates
        qjoint = configuration(jointState)
        global_coordinates!(jointState.joint, qjoint, q0joint, ϕjoint)
    end
end
