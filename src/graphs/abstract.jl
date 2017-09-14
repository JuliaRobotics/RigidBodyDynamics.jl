# Vertex interface
vertex_index(::Any) = error("Vertex types must implement this method")
vertex_index!(::Any, index::Int64) = error("Vertex types must implement this method")

# Edge interface
edge_index(::Any) = error("Edge types must implement this method")
edge_index!(::Any, index::Int64) = error("Edge types must implement this method")
flip_direction(x::Any) = deepcopy(x)

abstract type AbstractGraph{V, E} end
num_vertices(g::AbstractGraph) = length(vertices(g))
num_edges(g::AbstractGraph) = length(edges(g))
out_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (target(e, g) for e in out_edges(vertex, g))
in_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (source(e, g) for e in in_edges(vertex, g))
