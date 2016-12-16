module RBDCodeGen

using RigidBodyDynamics
using StaticArrays
using RigidBodyDynamics.TreeDataStructure
import RigidBodyDynamics: VectorSegment
import RigidBodyDynamics: num_positions, num_velocities

export
    MechanismState,
    update_joint_transforms,
    update_crb_inertias,
    mass_matrix!


type StateElement{R<:RigidBody, J<:Joint, X<:Real, C<:Real}
    body::R
    joint::J
    q::VectorSegment{X}
    v::VectorSegment{X}
    beforeJointToParent::Transform3D{C}
    jointTransform::Transform3D{C}
    transformToWorld::Transform3D{C}
    jointTwist::Twist{C}
    twistWrtWorld::Twist{C}
    biasAcceleration::SpatialAcceleration{C}
    crbInertia::SpatialInertia{C}

    function StateElement(body::R, joint::J, q::VectorSegment{X}, v::VectorSegment{X}, beforeJointToParent::Transform3D{C})
        new(body, joint, q, v, beforeJointToParent)
    end
end

function StateElement{R<:RigidBody, J<:Joint, X<:Real}(body::R, joint::J, q::VectorSegment{X}, v::VectorSegment{X}, beforeJointToParent::Transform3D)
    C = promote_type(eltype(R), eltype(J), X)
    StateElement{R, J, X, C}(body, joint, q, v, convert(Transform3D{C}, beforeJointToParent))
end

velocity_range(element::StateElement) = first(parentindexes(element.v))

type MechanismState{X <: Real, Es, Parents}
    mechanism::Mechanism
    q::Vector{X}
    v::Vector{X}
    elements::Es
    parents::Parents
end

state_elements_type{X, Es, Parents}(::Type{MechanismState{X, Es, Parents}}) = Es
num_elements{T<:MechanismState}(::Type{T}) = length(state_elements_type(T).parameters)
Base.parent{X, Es, Parents}(::Type{MechanismState{X, Es, Parents}}, i::Int64) = Parents.parameters[i].parameters[1]

function MechanismState{X <: Real}(::Type{X}, mechanism::Mechanism)
    q = Vector{X}(num_positions(mechanism))
    v = zeros(X, num_velocities(mechanism))
    elements = StateElement[]
    parents = Int64[]

    qStart, vStart = 1, 1
    for vertex in filter(v -> !isroot(v), toposort(tree(mechanism)))
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        parentIndex = findfirst(e -> e.body == vertex_data(parent(vertex)), elements)
        push!(parents, parentIndex)
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        beforeJointToParent = mechanism.jointToJointTransforms[joint]
        element = StateElement(body, joint, view(q, qStart : qEnd), view(v, vStart : vEnd), beforeJointToParent)
        push!(elements, element)
        qStart, vStart = qEnd + 1, vEnd + 1
        zero_configuration!(joint, element.q)
    end
    MechanismState(mechanism, q, v, tuple(elements...), tuple((Val{p}() for p in parents)...))
end

@generated function update_joint_transforms(state::MechanismState)
    exprs = [quote
        let
            element = state.elements[$i]
            element.jointTransform = element.beforeJointToParent * joint_transform(element.joint, element.q)
        end
    end for i = 1 : num_elements(state)]
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

@generated function update_crb_inertias(state::MechanismState)
    exprs = [quote
        let
            element = state.elements[$i]
            element.crbInertia = spatial_inertia(element.body)
        end
    end for i = 1 : num_elements(state)]

    for i = num_elements(state) : -1 : 1
        j = parent(state, i)
        if j != 0
            push!(exprs, quote
                toParent = state.elements[$i].jointTransform
                state.elements[$j].crbInertia += transform(state.elements[$i].crbInertia, toParent)
            end)
        end
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

function set_submatrix!(mat::Symmetric, part::AbstractMatrix, rowStart::Int64, colStart::Int64)
    for i = 1 : size(part, 1)
        row = rowStart + i - 1
        for j = 1 : size(part, 2)
            col = colStart + j - 1
            mat.data[row, col] = part[i, j]
        end
    end
end

function _mass_matrix_part(out::Symmetric, F::MomentumMatrix, irange::UnitRange{Int64}, element::StateElement)
    Sj = motion_subspace(element.joint, element.q)
    Hij = F.angular' * Sj.angular + F.linear' * Sj.linear
    jrange = velocity_range(element)
    set_submatrix!(out, Hij, start(irange), start(jrange))
end

@generated function _mass_matrix_inner_loop{i}(out::Symmetric, state::MechanismState, F::MomentumMatrix, ::Val{i})
    exprs = Expr[]
    push!(exprs, Expr(:meta, :noinline))
    push!(exprs, :(irange = velocity_range(state.elements[$i])))
    j = i
    while parent(state, j) != 0
        push!(exprs, :(F = transform(F, state.elements[$j].jointTransform)))
        j = parent(state, j)
        push!(exprs, :(_mass_matrix_part(out, F, irange, state.elements[$j])))
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

momentum_matrix(element::StateElement) = element.crbInertia * motion_subspace(element.joint, element.q)

@generated function mass_matrix!(out::Symmetric, state::MechanismState)
    exprs = Expr[]
    push!(exprs, quote
        update_joint_transforms(state)
        update_crb_inertias(state)
        fill!(out.data, zero(eltype(out)))
    end)

    for i = 1 : num_elements(state)
        ival = Val{i}()
        push!(exprs, quote
            let
                element = state.elements[$i]
                F = momentum_matrix(element)
                irange = velocity_range(element)
                _mass_matrix_part(out, F, irange, element)
                _mass_matrix_inner_loop(out, state, F, $ival)
            end
        end)
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

end
