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

type MechanismVertex{J<:Joint, R<:RigidBody}
    joint::J
    body::R
    index::Int64
end

type StateElement{X<:Real, C<:Real}
    q::VectorSegment{X}
    v::VectorSegment{X}
    parentIndex::Int64
    beforeJointToParent::Transform3D{C}
    jointTransform::Transform3D{C}
    # afterJointToWorld::Transform3D{C}
    # jointTwist::Twist{C}
    # twistWrtWorld::Twist{C}
    # biasAcceleration::SpatialAcceleration{C}
    crbInertia::SpatialInertia{C}

    StateElement(q::VectorSegment{X}, v::VectorSegment{X}, parentIndex::Int64, beforeJointToParent::Transform3D{C}) = new(q, v, parentIndex, beforeJointToParent)
end
StateElement{X, C}(q::VectorSegment{X}, v::VectorSegment{X}, parentIndex::Int64, beforeJointToParent::Transform3D{C}) = StateElement{X, C}(q, v, parentIndex, beforeJointToParent)

configuration_range(element::StateElement) = first(parentindexes(element.q))
velocity_range(element::StateElement) = first(parentindexes(element.v))

type MechanismState{X<:Real, C<:Real, V, TypeIndices, ParentIndices}
    mechanism::Mechanism
    q::Vector{X}
    v::Vector{X}
    vertices::V
    elements::Vector{StateElement{X, C}}
end

mechanism_vertices_type{X, C, V, TypeIndices, Parents}(::Type{MechanismState{X, C, V, TypeIndices, Parents}}) = V
num_vertex_types{T<:MechanismState}(::Type{T}) = length(mechanism_vertices_type(T).parameters)
parent_index{X, C, V, TypeIndices, Parents}(::Type{MechanismState{X, C, V, TypeIndices, Parents}}, i::Int64) = Parents.parameters[i].parameters[1]
type_index{X, C, V, TypeIndices, Parents}(::Type{MechanismState{X, C, V, TypeIndices, Parents}}, i::Int64) = TypeIndices.parameters[i].parameters[1]

function MechanismState{X<:Real}(::Type{X}, mechanism::Mechanism)
    q = zeros(X, num_positions(mechanism))
    v = zeros(X, num_velocities(mechanism))

    # Create concrete-typed MechanismVertices and computed parent indices
    vertices = []
    parentIndices = Int64[]
    C = X
    for (index, vertex) in enumerate(non_root_vertices(mechanism))
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        push!(vertices, MechanismVertex(joint, body, index))
        C = promote_type(C, eltype(joint), eltype(joint))
        parentIndex = findfirst(v -> v.body == vertex_data(parent(vertex)), vertices)
        push!(parentIndices, parentIndex)
    end

    # Sort vertices by type
    vertexTypes = unique(typeof(v) for v in vertices)
    sortedVertices = tuple((t[] for t in vertexTypes)...)
    typeIndices = Int64[]
    for vertex in vertices
        for (i, t) in enumerate(vertexTypes)
            if typeof(vertex) == t
                push!(sortedVertices[i], vertex)
                push!(typeIndices, i)
            end
        end
    end

    # Create state elements
    qStart, vStart = 1, 1
    stateElements = StateElement{X, C}[]
    for (index, vertex) in enumerate(non_root_vertices(mechanism))
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        qEnd, vEnd = qStart + num_positions(joint) - 1, vStart + num_velocities(joint) - 1
        beforeJointToParent = convert(Transform3D{C}, mechanism.jointToJointTransforms[joint])
        parentIndex = parentIndices[index]
        element = StateElement(view(q, qStart : qEnd), view(v, vStart : vEnd), parentIndex, beforeJointToParent)
        push!(stateElements, element)
        qStart, vStart = qEnd + 1, vEnd + 1
        zero_configuration!(joint, element.q)
    end

    V = typeof(sortedVertices)
    TypeIndices = typeof(tuple((Val{t}() for t in typeIndices)...))
    ParentIndices = typeof(tuple((Val{p}() for p in parentIndices)...))
    MechanismState{X, C, V, TypeIndices, ParentIndices}(mechanism, q, v, sortedVertices, stateElements)
end

@generated function update_joint_transforms(state::MechanismState)
    exprs = Expr[]
    for i = 1 : num_vertex_types(state)
        expr = quote
            let
                vertices = state.vertices[$i]
                for vertex in vertices
                    index = vertex.index
                    element = state.elements[index]
                    element.jointTransform = element.beforeJointToParent * joint_transform(vertex.joint, element.q)
                end
            end
        end
        push!(exprs, expr)
    end
    push!(exprs, :(return nothing))
    Expr(:block, exprs...)
end

@generated function update_crb_inertias(state::MechanismState)
    exprs = Expr[]

    # Initialize CRB inertias to body inertias
    for j = 1 : num_vertex_types(state)
        expr = quote
            let
                vertices = state.vertices[$j]
                for vertex in vertices
                    index = vertex.index
                    element = state.elements[index]
                    element.crbInertia = spatial_inertia(vertex.body)
                end
            end
        end
        push!(exprs, expr)
    end

    # Backwards pass
    expr = quote
        elements = state.elements
        for i = length(elements) : -1 : 1
            elementI = elements[i]
            j = elementI.parentIndex
            if j != 0
                elements[j].crbInertia += transform(elementI.crbInertia, elementI.jointTransform)
            end
        end
    end
    push!(exprs, expr)
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

# @generated function _mass_matrix_inner_loop{i}(out::Symmetric, state::MechanismState, F::MomentumMatrix, ::Val{i})
#     exprs = Expr[]
#     push!(exprs, Expr(:meta, :noinline))
#     push!(exprs, :(irange = velocity_range(state.elements[$i])))
#     j = i
#     while parent(state, j) != 0
#         push!(exprs, :(F = transform(F, state.elements[$j].jointTransform)))
#         j = parent(state, j)
#         push!(exprs, quote
#             let
#                 Sj = motion_subspace(state.elements[$j].joint, state.elements[$j].q)
#                 Hij = F.angular' * Sj.angular + F.linear' * Sj.linear
#                 jrange = velocity_range(state.elements[$j])
#                 set_submatrix!(out, Hij, start(irange), start(jrange))
#             end
#         end)
#     end
#     push!(exprs, :(return nothing))
#     Expr(:block, exprs...)
# end
#
# @generated function mass_matrix!(out::Symmetric, state::MechanismState)
#     exprs = Expr[]
#     push!(exprs, quote
#         update_joint_transforms(state)
#         update_crb_inertias(state)
#         fill!(out.data, zero(eltype(out)))
#     end)
#
#     for i = 1 : num_elements(state)
#         ival = Val{i}()
#         push!(exprs, quote
#             let
#                 elementi = state.elements[$i]
#                 Ii = elementi.crbInertia
#                 Si = motion_subspace(elementi.joint, elementi.q)
#                 F = Ii * Si
#                 Hii =  F.angular' * Si.angular + F.linear' * Si.linear
#                 irange = velocity_range(elementi)
#                 set_submatrix!(out, Hii, start(irange), start(irange))
#                 _mass_matrix_inner_loop(out, state, F, $ival)
#             end
#         end)
#     end
#     push!(exprs, :(return nothing))
#     Expr(:block, exprs...)
# end

end # module
