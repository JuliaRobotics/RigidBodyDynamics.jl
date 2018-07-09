mutable struct DirectedGraph{V, E} <: AbstractGraph{V, E}
    vertices::Vector{V}
    edges::Vector{E}
    sources::Vector{V}
    targets::Vector{V}
    inedges::Vector{Vector{E}}
    outedges::Vector{Vector{E}}
end

DirectedGraph{V, E}() where {V, E} = DirectedGraph{V, E}(V[], E[], V[], V[], Set{E}[], Set{E}[])

function DirectedGraph(vertexfun::Base.Callable, edgefun::Base.Callable, other::DirectedGraph)
    vertices = map(vertexfun, other.vertices)
    edges = map(edgefun, other.edges)
    vertexmap = Dict(zip(other.vertices, vertices))
    edgemap = Dict(zip(other.edges, edges))
    sources = [vertexmap[v] for v in other.sources]
    targets = [vertexmap[v] for v in other.targets]
    inedges = [[edgemap[e] for e in vec] for vec in other.inedges]
    outedges = [[edgemap[e] for e in vec] for vec in other.outedges]
    DirectedGraph(vertices, edges, sources, targets, inedges, outedges)
end

# AbstractGraph interface
vertices(g::DirectedGraph) = g.vertices
edges(g::DirectedGraph) = g.edges
source(edge, g::DirectedGraph{V, E}) where {V, E} = g.sources[edge_index(edge)]
target(edge, g::DirectedGraph{V, E}) where {V, E} = g.targets[edge_index(edge)]
in_edges(vertex::V, g::DirectedGraph{V, E}) where {V, E} = g.inedges[vertex_index(vertex)]
out_edges(vertex::V, g::DirectedGraph{V, E}) where {V, E} = g.outedges[vertex_index(vertex)]

Base.show(io::IO, ::DirectedGraph{V, E}) where {V, E} = print(io, "DirectedGraph{$V, $E}(…)")

function Base.empty!(g::DirectedGraph)
    empty!(g.vertices)
    empty!(g.edges)
    empty!(g.sources)
    empty!(g.targets)
    empty!(g.inedges)
    empty!(g.outedges)
    g
end

function add_vertex!(g::DirectedGraph{V, E}, vertex::V) where {V, E}
    @assert vertex ∉ vertices(g)
    vertex_index!(vertex, num_vertices(g) + 1)
    push!(g.vertices, vertex)
    push!(g.outedges, E[])
    push!(g.inedges, E[])
    g
end

function add_edge!(g::DirectedGraph{V, E}, source::V, target::V, edge::E) where {V, E}
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

function remove_vertex!(g::DirectedGraph{V, E}, vertex::V) where {V, E}
    disconnected = isempty(in_edges(vertex, g)) && isempty(out_edges(vertex, g))
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

function remove_edge!(g::DirectedGraph{V, E}, edge::E) where {V, E}
    target_inedges = in_edges(target(edge, g), g)
    deleteat!(target_inedges, findfirst(isequal(edge), target_inedges))
    source_outedges = out_edges(source(edge, g), g)
    deleteat!(source_outedges, findfirst(isequal(edge), source_outedges))
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

function rewire!(g::DirectedGraph{V, E}, edge::E, newsource::V, newtarget::V) where {V, E}
    oldsource = source(edge, g)
    oldtarget = target(edge, g)

    g.sources[edge_index(edge)] = newsource
    g.targets[edge_index(edge)] = newtarget

    oldtarget_inedges = in_edges(oldtarget, g)
    deleteat!(oldtarget_inedges, findfirst(isequal(edge), oldtarget_inedges))
    oldsource_outedges = out_edges(oldsource, g)
    deleteat!(oldsource_outedges, findfirst(isequal(edge), oldsource_outedges))

    push!(out_edges(newsource, g), edge)
    push!(in_edges(newtarget, g), edge)

    g
end

function replace_edge!(g::DirectedGraph{V, E}, old_edge::E, new_edge::E) where {V, E}
    if new_edge !== old_edge
        src = source(old_edge, g)
        dest = target(old_edge, g)
        index = edge_index(old_edge)
        edge_index!(new_edge, index)
        g.edges[index] = new_edge
        out_edge_index = findfirst(isequal(old_edge), out_edges(src, g))
        out_edges(src, g)[out_edge_index] = new_edge
        in_edge_index = findfirst(isequal(old_edge), in_edges(dest, g))
        in_edges(dest, g)[in_edge_index] = new_edge
        edge_index!(old_edge, -1)
    end
    g
end

function reindex!(g::DirectedGraph{V, E}, vertices_in_order, edges_in_order) where {V, E}
    @assert isempty(setdiff(vertices(g), vertices_in_order))
    @assert isempty(setdiff(edges(g), edges_in_order))
    sources = source.(edges_in_order, Ref(g))
    targets = target.(edges_in_order, Ref(g))
    empty!(g)
    for v in vertices_in_order
        add_vertex!(g, v)
    end
    for (source, target, e) in zip(sources, targets, edges_in_order)
        add_edge!(g, source, target, e)
    end
    g
end
