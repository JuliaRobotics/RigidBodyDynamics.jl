module Graphs

using Compat

export
    DirectedGraph,
    SpanningTree,
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
out_neighbors{V, E}(vertex::V, g::AbstractGraph{V, E}) = (target(e) for e in out_edges(v, g))
in_neighbors{V, E}(vertex::V, g::AbstractGraph{V, E}) = (source(e) for e in in_edges(v, g))

type DirectedGraph{V, E}
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

function rewire!{V, E}(edge::E, newsource::V, newtarget::V, g::DirectedGraph{V, E})
    oldsource = source(edge, g)
    oldtarget = target(edge, g)

    g.sources[edge_index(edge)] = newsource
    g.targets[edge_index(edge)] = newtarget

    delete!(out_edges(oldsource, g), edge)
    delete!(in_edges(oldtarget, g), edge)

    push!(out_edges(newsource, g), edge)
    push!(in_edges(newtarget, g), edge)

    nothing
end

function flip_direction!{V, E}(edge::E, g::DirectedGraph{V, E})
    flip_direction!(edge)
    rewire!(edge, target(edge, g), source(edge, g), g)
end

# function edge_between{V, E}(from::V, to::V, g::DirectedGraph{V, E})
#     for edge in in_edges(target, g)
#         source(edge, g) == from && return edge
#     end
#     error("Edge not found")
# end

type SpanningTree{V, E}
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

root(tree::SpanningTree) = source(first(edges(tree)))
edge_to_parent{V, E}(vertex::V, tree::SpanningTree{V, E}) = tree.inedges[vertex_index(vertex)]
edges_to_children{V, E}(vertex::V, tree::SpanningTree{V, E}) = out_edges(vertex, tree)

function SpanningTree{V, E}(g::DirectedGraph{V, E}, root::V, next_edge = (graph, frontier) -> first(frontier))
    n = num_vertices(g)
    ret = SpanningTree(g, E[], Vector{E}(n), [Set{E}() for i = 1 : n])
    treevertices = [root]
    frontier = E[]
    append!(frontier, out_edges(root, g))
    while !isempty(frontier)
        # select a new edge
        e = next_edge(g, frontier)
        parent = source(e, g)
        child = target(e, g)

        # add edge to tree
        push!(tree.edges, e)
        tree.inedges[vertex_index(child)] = e
        push!(edges_to_children(parent), e)

        # update the frontier
        push!(treevertices, child)
        filter!(x -> x ∈ in_edges(child, g), frontier)
        append!(frontier, x for x in out_edges(child, g) if target(x, g) ∉ treevertices)
    end
    length(treevertices) == num_vertices(g) || error("Could not construct a spanning tree.")
    ret
end

# adds an edge and vertex to both the tree and the underlying graph
function add_edge!{V, E}(tree::SpanningTree{V, E}, source::V, target::V, edge::E)
    @assert target ∉ vertices(tree)
    add_edge!(tree.graph, source, target, edge)
    push!(tree.edges, edge)
    push!(tree.inedges, edge)
    push!(out_edges(source, tree), edge)
    tree
end


end # module
