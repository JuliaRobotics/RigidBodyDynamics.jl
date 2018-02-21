# Vertex interface
"""
    vertex_id(vertex)

Return an identifier of the vertex. The identifier must be of a type that can be converted from and to an Int.
"""
function vertex_id end

"""
    set_vertex_id!(vertex, id)

Set the vertex identifier. The identifier must be of a type that can be converted from and to an Int.
"""
function set_vertex_id! end

"""
    vertex_id_type(V)

Return the identifier type used by vertex type `V`. The identifier type must be convertible from and to an Int.
"""
vertex_id_type(::Type) = Int # fallback, can be specialized

@inline vertex_index(x) = Int(vertex_id(x))
@inline vertex_index!(x::T, index::Int) where {T} = set_vertex_id!(x, vertex_id_type(T)(index))


# Edge interface
"""
    edge_id(edge)

Return an identifier of the edge. The identifier must be of a type that can be converted from and to an Int.
"""
function edge_id end

"""
    set_edge_id!(edge, id)

Set the edge identifier. The identifier must be of a type that can be converted from and to an Int.
"""
function set_edge_id! end

"""
    edge_id_type(V)

Return the identifier type used by edge type `V`. The identifier type must be convertible from and to an Int.
"""
edge_id_type(::Type) = Int # fallback, can be specialized

@inline edge_index(x) = Int(edge_id(x))
@inline edge_index!(x::T, index::Int) where {T} = set_edge_id!(x, edge_id_type(T)(index))

flip_direction(x::Any) = deepcopy(x)


abstract type AbstractGraph{V, E} end
vertextype(::Type{<:AbstractGraph{V, E}}) where {V, E} = V
vertextype(g::AbstractGraph) = vertextype(typeof(g))
edgetype(::Type{<:AbstractGraph{V, E}}) where {V, E} = E
edgetype(g::AbstractGraph) = edgetype(typeof(g))
num_vertices(g::AbstractGraph) = length(vertices(g))
num_edges(g::AbstractGraph) = length(edges(g))
out_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (target(e, g) for e in out_edges(vertex, g))
in_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (source(e, g) for e in in_edges(vertex, g))
