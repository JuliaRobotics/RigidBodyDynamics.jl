module Graphs

using Compat

export
    DirectedGraph,
    SpanningTree,
    Edge,
    Vertex,
    vertex_index,
    edge_index,
    vertex_type,
    edge_type,
    vertices,
    edges,
    source,
    target,
    out_edges,
    in_edges,
    num_vertices,
    num_edges,
    add_vertex!,
    add_edge!,
    remove_vertex!,
    remove_edge!,
    rewire!,
    root,
    edge_to_parent,
    edges_to_children

# vertex interface
vertex_index(::Any) = error("Vertex types must implement this method")
vertex_index!(::Any, index::Int64) = error("Vertex types must implement this method")

# edge interface
edge_index(::Any) = error("Edge types must implement this method")
edge_index!(::Any, index::Int64) = error("Edge types must implement this method")
flip_direction!(::Any) = nothing # optional

@compat abstract type AbstractGraph{V, E} end

vertex_type{V, E}(::Type{AbstractGraph{V, E}}) = V
edge_type{V, E}(::Type{AbstractGraph{V, E}}) = E
num_vertices(g::AbstractGraph) = length(vertices(g))
num_edges(g::AbstractGraph) = length(edges(g))
out_neighbors{V, E}(vertex::V, g::AbstractGraph{V, E}) = (target(e, g) for e in out_edges(v, g))
in_neighbors{V, E}(vertex::V, g::AbstractGraph{V, E}) = (source(e, g) for e in in_edges(v, g))

type DirectedGraph{V, E} <: AbstractGraph{V, E}
    vertices::Vector{V}
    edges::Vector{E}
    sources::Vector{V}
    targets::Vector{V}
    inedges::Vector{Set{E}}
    outedges::Vector{Set{E}}

    DirectedGraph() = new(V[], E[], V[], V[], Set{E}[], Set{E}[])
end

# renumber function?

# AbstractGraph interface
vertices(g::DirectedGraph) = g.vertices
edges(g::DirectedGraph) = g.edges
source{V, E}(edge::E, g::DirectedGraph{V, E}) = g.sources[edge_index(edge)]
target{V, E}(edge::E, g::DirectedGraph{V, E}) = g.targets[edge_index(edge)]
in_edges{V, E}(vertex::V, g::DirectedGraph{V, E}) = g.inedges[vertex_index(vertex)]
out_edges{V, E}(vertex::V, g::DirectedGraph{V, E}) = g.outedges[vertex_index(vertex)]

Base.show{V, E}(io::IO, ::DirectedGraph{V, E}) = print(io, "DirectedGraph{$V, $E}(…)")

function add_vertex!{V, E}(g::DirectedGraph{V, E}, vertex::V)
    @assert vertex ∉ vertices(g)

    vertex_index!(vertex, num_vertices(g) + 1)
    push!(g.vertices, vertex)
    push!(g.outedges, Set{E}())
    push!(g.inedges, Set{E}())
    g
end

function add_edge!{V, E}(g::DirectedGraph{V, E}, source::V, target::V, edge::E)
    @assert edge ∉ edges(g)
    source ∈ vertices(g) || add_vertex!(g, source)
    target ∈ vertices(g) || add_vertex!(g, target)

    edge_index!(edge, num_edges(g) + 1)
    push!(g.edges, edge)
    push!(g.sources, source)
    push!(g.targets, target)
    push!(out_edges(source, g), edge)
    push!(in_edges(target, g), edge)
    g
end

# function depth_first_search{V, E}(g::DirectedGraph{V, E}, root::V, ignoredirection = false)
#     stack = [root]
#     discovered = V[]
#     while !isempty(stack)
#         v = pop!(stack)
#         if v ∉ discovered
#             push!(discovered, v)
#             append!(stack, out_neighbors(v, g))
#             if ignoredirection
#                 append!(stack, in_neighbors(v, g))
#             end
#         end
#     end
#     discovered
# end

function remove_vertex!{V, E}(g::DirectedGraph{V, E}, vertex::V)
    disconnected = isempty(in_edges(vertex, g)) && isempty(out_edges(vertex, g))
    if !disconnected
        @show in_edges(vertex, g)
        @show out_edges(vertex, g)
    end
    disconnected || error("Vertex must be disconnected from the rest of the graph before it can be removed.")
    index = vertex_index(vertex)
    deleteat!(g.vertices, index)
    deleteat!(g.inedges, index)
    deleteat!(g.outedges, index)
    for i = index : num_vertices(g)
        vertex_index!(g.vertices[i], i)
    end
    vertex_index!(vertex, -1)
    g
end

function remove_edge!{V, E}(g::DirectedGraph{V, E}, edge::E)
    delete!(in_edges(target(edge, g), g), edge)
    delete!(out_edges(source(edge, g), g), edge)
    index = edge_index(edge)
    deleteat!(g.edges, index)
    deleteat!(g.sources, index)
    deleteat!(g.targets, index)
    for i = index : num_edges(g)
        edge_index!(g.edges[i], i)
    end
    edge_index!(edge, -1)
    g
end

function rewire!{V, E}(g::DirectedGraph{V, E}, edge::E, newsource::V, newtarget::V)
    oldsource = source(edge, g)
    oldtarget = target(edge, g)

    g.sources[edge_index(edge)] = newsource
    g.targets[edge_index(edge)] = newtarget

    delete!(out_edges(oldsource, g), edge)
    delete!(in_edges(oldtarget, g), edge)

    push!(out_edges(newsource, g), edge)
    push!(in_edges(newtarget, g), edge)

    g
end

function flip_direction!{V, E}(edge::E, g::DirectedGraph{V, E})
    flip_direction!(edge)
    rewire!(g, edge, target(edge, g), source(edge, g))
end

# function edge_between{V, E}(from::V, to::V, g::DirectedGraph{V, E})
#     for edge in in_edges(target, g)
#         source(edge, g) == from && return edge
#     end
#     error("Edge not found")
# end

type SpanningTree{V, E} <: AbstractGraph{V, E}
    graph::DirectedGraph{V, E}
    edges::Vector{E}
    inedges::Vector{E}
    outedges::Vector{Set{E}}
end

# AbstractGraph interface
vertices(tree::SpanningTree) = vertices(tree.graph)
edges(tree::SpanningTree) = tree.edges
source{V, E}(edge::E, tree::SpanningTree{V, E}) = source(edge, tree.graph) # note: doesn't check that edge is in spanning tree!
target{V, E}(edge::E, tree::SpanningTree{V, E}) = target(edge, tree.graph) # note: doesn't check that edge is in spanning tree!
in_edges{V, E}(vertex::V, tree::SpanningTree{V, E}) = (tree.inedges[vertex_index(vertex)],)
out_edges{V, E}(vertex::V, tree::SpanningTree{V, E}) = tree.outedges[vertex_index(vertex)]

# TODO: Base.show

root(tree::SpanningTree) = source(first(edges(tree)), tree)
edge_to_parent{V, E}(vertex::V, tree::SpanningTree{V, E}) = tree.inedges[vertex_index(vertex)]
edges_to_children{V, E}(vertex::V, tree::SpanningTree{V, E}) = out_edges(vertex, tree)

function SpanningTree{V, E}(g::DirectedGraph{V, E}, edges::AbstractVector{E})
    n = num_vertices(g)
    length(edges) == n - 1 || error("Expected n - 1 edges.")
    inedges = Vector{E}(n)
    outedges = [Set{E}() for i = 1 : n]
    treevertices = V[]
    for edge in edges
        parent = source(edge, g)
        child = target(edge, g)
        isempty(treevertices) && push!(treevertices, parent)
        @assert parent ∈ treevertices
        inedges[vertex_index(child)] = edge
        push!(outedges[vertex_index(parent)], edge)
        push!(treevertices, child)
    end
    SpanningTree(g, edges, inedges, outedges)
end

function SpanningTree{V, E}(g::DirectedGraph{V, E}, root::V, next_edge = (graph, frontier) -> first(frontier))
    treevertices = [root]
    edges = E[]
    frontier = E[]
    append!(frontier, out_edges(root, g))
    while !isempty(frontier)
        # select a new edge
        e = next_edge(g, frontier)
        parent = source(e, g)
        child = target(e, g)

        # add edge to tree
        push!(edges, e)

        # update the frontier
        push!(treevertices, child)
        filter!(x -> x ∉ in_edges(child, g), frontier)
        append!(frontier, x for x in out_edges(child, g) if target(x, g) ∉ treevertices)
    end
    SpanningTree(g, edges)
end

# adds an edge and vertex to both the tree and the underlying graph
function add_edge!{V, E}(tree::SpanningTree{V, E}, source::V, target::V, edge::E)
    @assert target ∉ vertices(tree)
    add_edge!(tree.graph, source, target, edge)
    push!(tree.edges, edge)
    push!(tree.inedges, edge)
    push!(tree.outedges, Set{E}())
    push!(out_edges(source, tree), edge)
    tree
end

function Base.show(io::IO, tree::SpanningTree, vertex = root(tree), level::Int64 = 0)
    for i = 1 : level print(io, "  ") end
    print(io, "Vertex: ")
    showcompact(io, vertex)
    if vertex == root(tree)
        print(io, " (root)")
    else
        print(io, ", ")
        print(io, "Edge: ")
        showcompact(io, edge_to_parent(vertex, tree))
    end
    for edge in edges_to_children(vertex, tree)
        print(io, "\n")
        child = target(edge, tree)
        show(io, tree, child, level + 1)
    end
end

# Useful for wrapping an existing type with the edge interface.
type Edge{T}
    data::T
    id::Int64

    Edge(data::T) = new(data, -1)
end
edge_index(edge::Edge) = edge.id
edge_index!(edge::Edge, index::Int64) = (edge.id = index)

# Useful for wrapping an existing type with the vertex interface.
type Vertex{T}
    data::T
    id::Int64

    Vertex(data::T) = new(data, -1)
end
vertex_index(vertex::Vertex) = vertex.id
vertex_index!(vertex::Vertex, index::Int64) = (vertex.id = vertex)

end # module
