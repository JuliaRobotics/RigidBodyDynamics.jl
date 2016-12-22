module RBDCodeGen

using RigidBodyDynamics
using StaticArrays
using RigidBodyDynamics.TreeDataStructure
import RigidBodyDynamics: VectorSegment
import RigidBodyDynamics: num_positions, num_velocities, @framecheck

export
    MechanismState,
    update_joint_transforms,
    update_crb_inertias,
    mass_matrix!

type StateElement{X<:Real, C<:Real}
    q::VectorSegment{X}
    v::VectorSegment{X}
    parentIndex::Int64
    beforeJointToParent::Transform3D{C}
    jointTransform::Transform3D{C}
    transformToWorld::Transform3D{C}
    jointTwist::Twist{C}
    twistWrtWorld::Twist{C}
    biasAcceleration::SpatialAcceleration{C}
    crbInertia::SpatialInertia{C}

    StateElement(q::VectorSegment{X}, v::VectorSegment{X}, parentIndex::Int64, beforeJointToParent::Transform3D{C}) = new(q, v, parentIndex, beforeJointToParent)
end
StateElement{X, C}(q::VectorSegment{X}, v::VectorSegment{X}, parentIndex::Int64, beforeJointToParent::Transform3D{C}) = StateElement{X, C}(q, v, parentIndex, beforeJointToParent)
velocity_range(element::StateElement) = first(parentindexes(element.v))

type MechanismVertex{R<:RigidBody, J<:Joint}
    body::R
    joint::J
    index::Int64
end

type MechanismState{X <: Real, C<:Real, V, Parents}
    mechanism::Mechanism
    q::Vector{X}
    v::Vector{X}
    vertices::V
    elements::Vector{StateElement{X, C}}
end

Base.@pure vertices_type{X, C, V, Parents}(::Type{MechanismState{X, C, V, Parents}}) = V
Base.@pure num_vertices{T<:MechanismState}(::Type{T}) = length(vertices_type(T).parameters)
parent_index{X, C, V, Parents}(::Type{MechanismState{X, C, V, Parents}}, i::Int64) = Parents.parameters[i].parameters[1]

function MechanismState{X <: Real}(::Type{X}, mechanism::Mechanism)
    q = Vector{X}(num_positions(mechanism))
    v = zeros(X, num_velocities(mechanism))
    vertices = MechanismVertex[]
    parents = Int64[]
    C = X
    nonRootVertices = filter(v -> !isroot(v), toposort(tree(mechanism)))
    for (index, vertex) in enumerate(nonRootVertices)
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        parentIndex = findfirst(vert -> vert.body == vertex_data(parent(vertex)), vertices)
        push!(parents, parentIndex)
        vertex = MechanismVertex(body, joint, index)
        push!(vertices, vertex)
        C = promote_type(C, eltype(typeof(body)), eltype(typeof(joint)))
    end

    qStart, vStart = 1, 1
    elements = StateElement{X, C}[]
    for (index, vertex) in enumerate(nonRootVertices)
        joint = edge_to_parent_data(vertex)
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        beforeJointToParent = mechanism.jointToJointTransforms[joint]
        element = StateElement(view(q, qStart : qEnd), view(v, vStart : vEnd), parents[index], convert(Transform3D{C}, beforeJointToParent))
        push!(elements, element)
        qStart, vStart = qEnd + 1, vEnd + 1
        zero_configuration!(joint, element.q)
    end

    verticesTuple = tuple(vertices...)
    parentsTuple = tuple((Val{p}() for p in parents)...)
    MechanismState{X, C, typeof(verticesTuple), typeof(parentsTuple)}(mechanism, q, v, vertices, elements)
end

@generated function update_joint_transforms(state::MechanismState)
    exprs = Expr[]
    for i = 1 : num_vertices(state)
        push!(exprs, quote
            element = state.elements[$i]
            element.jointTransform = element.beforeJointToParent * joint_transform(state.vertices[$i].joint, element.q)
        end)
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

@generated function update_crb_inertias(state::MechanismState)
    exprs = [quote
        let
            state.elements[$i].crbInertia = spatial_inertia(state.vertices[$i].body)
        end
    end for i = 1 : num_vertices(state)]

    push!(exprs, quote
        for i = length(state.elements) : -1 : 1
            element = state.elements[i]
            if element.parentIndex != 0
                state.elements[element.parentIndex].crbInertia += transform(element.crbInertia, element.jointTransform)
            end
        end
    end)
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

@generated function mass_matrix!(out::Symmetric, state::MechanismState)
    mass_matrix!_gen_impl(out, state)
end

function mass_matrix_part(F::MomentumMatrix, S::GeometricJacobian)
    @framecheck F.frame S.frame
    F.angular' * S.angular + F.linear' * S.linear
end

function mass_matrix!_gen_impl(out, state)
    exprs = Expr[]
    push!(exprs, quote
        update_joint_transforms(state)
        update_crb_inertias(state)
        fill!(out.data, zero(eltype(out)))
    end)

    for i = 1 : num_vertices(state)
        ival = Val{i}
        push!(exprs, quote
            element = state.elements[$i]
            irange = velocity_range(element)
            Ii = element.crbInertia
            let
                local Si = motion_subspace(state.vertices[$i].joint, element.q)
                local F = Ii * Si
                local Hii = mass_matrix_part(F, Si)
                set_submatrix!(out, Hii, start(irange), start(irange))
                _mass_matrix_inner_loop(out, state, F, irange, $ival)
            end
        end)
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

@generated function _mass_matrix_inner_loop{i}(out::Symmetric, state::MechanismState, F::MomentumMatrix, irange::UnitRange{Int64}, ::Type{Val{i}})
    exprs = Expr[]
    j = i
    while parent_index(state, j) != 0
        push!(exprs, quote
            element = state.elements[$j]
            F = transform(F, element.jointTransform)
        end)
        j = parent_index(state, j)
        push!(exprs, quote
            element = state.elements[$j]
            jrange = velocity_range(element)
            let
                local Sj = motion_subspace(state.vertices[$j].joint, element.q)
                local Hij = mass_matrix_part(F, Sj)
                set_submatrix!(out, Hij, start(irange), start(jrange))
            end
        end)
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

end
