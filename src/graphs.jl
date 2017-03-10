module Graphs

export
    DirectedGraph,
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
    add_edge!

# vertex interface
vertex_index(::Any) = error("Vertex types must implement this method")
vertex_index!(::Any, index::Int64) = error("Vertex types must implement this method")

# edge interface
edge_index(::Any) = error("Edge types must implement this method")
edge_index!(::Any, index::Int64) = error("Edge types must implement this method")

type DirectedGraph{V, E}
    vertices::Vector{V}
    edges::Vector{E}
    sources::Vector{V}
    targets::Vector{V}
    outedges::Vector{Set{E}}
    inedges::Vector{Set{E}}

    DirectedGraph() = new(V[], E[], V[], V[], Set{E}[], Set{E}[])
end

vertex_type{V, E}(::Type{DirectedGraph{V, E}}) = V
edge_type{V, E}(::Type{DirectedGraph{V, E}}) = E
vertices(g::DirectedGraph) = g.vertices
edges(g::DirectedGraph) = g.edges
num_vertices(g::DirectedGraph) = length(vertices(g))
num_edges(g::DirectedGraph) = length(edges(g))
source{V, E}(edge::E, g::DirectedGraph{V, E}) = g.sources[edge_index(edge)]
target{V, E}(edge::E, g::DirectedGraph{V, E}) = g.targets[edge_index(edge)]
out_edges{V, E}(vertex::V, g::DirectedGraph{V, E}) = g.outedges[vertex_index(vertex)]
in_edges{V, E}(vertex::V, g::DirectedGraph{V, E}) = g.inedges[vertex_index(vertex)]
out_neighbors{V, E}(vertex::V, g::DirectedGraph{V, E}) = (target(e) for e in out_edges(v, g))
in_neighbors{V, E}(vertex::V, g::DirectedGraph{V, E}) = (source(e) for e in in_edges(v, g))

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

function depth_first_search{V, E}(g::DirectedGraph{V, E}, root::V)
    # treats graph as being undirected
    stack = [root]
    discovered = V[]
    while !isempty(stack)
        v = pop!(stack)
        if v ∉ discovered
            push!(discovered, v)
            append!(stack, in_neighbors(v, g))
            append!(stack, out_neighbors(v, g))
        end
    end
    discovered
end

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

flip_direction!{V, E}(edge::E, g::DirectedGraph{V, E}) = rewire!(edge, target(edge, g), source(edge, g), g)

end # module
