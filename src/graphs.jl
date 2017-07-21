module Graphs

import Base.Iterators: flatten
import RigidBodyDynamics: UnsafeFastDict

export
    DirectedGraph,
    SpanningTree,
    TreePath,
    Edge,
    Vertex,
    data,
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
    out_neighbors,
    in_neighbors,
    num_vertices,
    num_edges,
    add_vertex!,
    add_edge!,
    remove_vertex!,
    remove_edge!,
    rewire!,
    flip_direction!,
    replace_edge!,
    root,
    edge_to_parent,
    edges_to_children,
    tree_index,
    ancestors,
    lowest_common_ancestor,
    path, # deprecated
    direction

# Vertex interface
vertex_index(::Any) = error("Vertex types must implement this method")
vertex_index!(::Any, index::Int64) = error("Vertex types must implement this method")

# Edge interface
edge_index(::Any) = error("Edge types must implement this method")
edge_index!(::Any, index::Int64) = error("Edge types must implement this method")
flip_direction!(::Any) = nothing # optional

# Vertex and Edge types; useful for wrapping an existing type with the edge interface.
# Note that DirectedGraph does not require using these types; just implement the edge interface.
for typename in (:Edge, :Vertex)
    getid = Symbol(lowercase(string(typename)) * "_index")
    setid = Symbol(lowercase(string(typename)) * "_index!")
    @eval begin
        mutable struct $typename{T}
            data::T
            id::Int64

            $typename(data::T) where {T} = new{T}(data, -1)
        end
        data(x::$typename) = x.data
        $getid(x::$typename) = x.id
        $setid(x::$typename, index::Int64) = (x.id = index)
    end
end

abstract type AbstractGraph{V, E} end
num_vertices(g::AbstractGraph) = length(vertices(g))
num_edges(g::AbstractGraph) = length(edges(g))
out_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (target(e, g) for e in out_edges(vertex, g))
in_neighbors(vertex::V, g::AbstractGraph{V, E}) where {V, E} = (source(e, g) for e in in_edges(vertex, g))


mutable struct DirectedGraph{V, E} <: AbstractGraph{V, E}
    vertices::Vector{V}
    edges::Vector{E}
    sources::Vector{V}
    targets::Vector{V}
    inedges::Vector{Vector{E}}
    outedges::Vector{Vector{E}}

    DirectedGraph{V, E}() where {V, E} = new{V, E}(V[], E[], V[], V[], Set{E}[], Set{E}[])
end

# AbstractGraph interface
vertices(g::DirectedGraph) = g.vertices
edges(g::DirectedGraph) = g.edges
source(edge, g::DirectedGraph{V, E}) where {V, E} = g.sources[edge_index(edge)]
target(edge, g::DirectedGraph{V, E}) where {V, E} = g.targets[edge_index(edge)]
in_edges(vertex::V, g::DirectedGraph{V, E}) where {V, E} = g.inedges[vertex_index(vertex)]
out_edges(vertex::V, g::DirectedGraph{V, E}) where {V, E} = g.outedges[vertex_index(vertex)]

Base.show(io::IO, ::DirectedGraph{V, E}) where {V, E} = print(io, "DirectedGraph{$V, $E}(…)")

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
    deleteat!(target_inedges, findfirst(target_inedges, edge))
    source_outedges = out_edges(source(edge, g), g)
    deleteat!(source_outedges, findfirst(source_outedges, edge))
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
    deleteat!(oldtarget_inedges, findfirst(oldtarget_inedges, edge))
    oldsource_outedges = out_edges(oldsource, g)
    deleteat!(oldsource_outedges, findfirst(oldsource_outedges, edge))

    push!(out_edges(newsource, g), edge)
    push!(in_edges(newtarget, g), edge)

    g
end

function flip_direction!(edge::E, g::DirectedGraph{V, E}) where {V, E}
    flip_direction!(edge)
    rewire!(g, edge, target(edge, g), source(edge, g))
end

function replace_edge!(g::DirectedGraph{V, E}, old_edge::E, new_edge::E) where {V, E}
    src = source(old_edge, g)
    dest = target(old_edge, g)
    index = edge_index(old_edge)
    edge_index!(new_edge, index)
    g.edges[index] = new_edge
    out_edge_index = findfirst(out_edges(src, g), old_edge)
    out_edges(src, g)[out_edge_index] = new_edge
    in_edge_index = findfirst(in_edges(dest, g), old_edge)
    in_edges(dest, g)[in_edge_index] = new_edge
    edge_index!(old_edge, -1)
    g
end

mutable struct SpanningTree{V, E} <: AbstractGraph{V, E}
    graph::DirectedGraph{V, E}
    root::V
    edges::Vector{E}
    inedges::Vector{E}
    outedges::Vector{Vector{E}}
    edge_tree_indices::Vector{Int64} # mapping from edge_index(edge) to index into edges(tree)
end

# AbstractGraph interface
vertices(tree::SpanningTree) = vertices(tree.graph)
edges(tree::SpanningTree) = tree.edges
source(edge, tree::SpanningTree{V, E}) where {V, E} = source(edge, tree.graph) # note: doesn't check that edge is in spanning tree!
target(edge, tree::SpanningTree{V, E}) where {V, E} = target(edge, tree.graph) # note: doesn't check that edge is in spanning tree!
in_edges(vertex::V, tree::SpanningTree{V, E}) where {V, E} = (tree.inedges[vertex_index(vertex)],)
out_edges(vertex::V, tree::SpanningTree{V, E}) where {V, E} = tree.outedges[vertex_index(vertex)]

root(tree::SpanningTree) = tree.root
edge_to_parent(vertex::V, tree::SpanningTree{V, E}) where {V, E} = tree.inedges[vertex_index(vertex)]
edges_to_children(vertex::V, tree::SpanningTree{V, E}) where {V, E} = out_edges(vertex, tree)
tree_index(edge, tree::SpanningTree{V, E}) where {V, E} = tree.edge_tree_indices[edge_index(edge)]
tree_index(vertex::V, tree::SpanningTree{V, E}) where {V, E} = vertex == root(tree) ? 1 : tree_index(edge_to_parent(vertex, tree), tree) + 1

function SpanningTree(g::DirectedGraph{V, E}, root::V, edges::AbstractVector{E}) where {V, E}
    n = num_vertices(g)
    length(edges) == n - 1 || error("Expected n - 1 edges.")
    inedges = Vector{E}(n)
    outedges = [E[] for i = 1 : n]
    edge_tree_indices = Int64[]
    treevertices = V[]
    for (i, edge) in enumerate(edges)
        parent = source(edge, g)
        child = target(edge, g)
        isempty(treevertices) && push!(treevertices, parent)
        @assert parent ∈ treevertices
        inedges[vertex_index(child)] = edge
        push!(outedges[vertex_index(parent)], edge)
        push!(treevertices, child)
        resize!(edge_tree_indices, max(edge_index(edge), length(edge_tree_indices)))
        edge_tree_indices[edge_index(edge)] = i
    end
    SpanningTree(g, root, edges, inedges, outedges, edge_tree_indices)
end

function SpanningTree(g::DirectedGraph{V, E}, root::V, next_edge = first #= breadth first =#) where {V, E}
    treevertices = [root]
    edges = E[]
    frontier = E[]
    append!(frontier, out_edges(root, g))
    append!(frontier, in_edges(root, g))
    while !isempty(frontier)
        # select a new edge
        e = next_edge(frontier)
        if source(e, g) ∉ treevertices
            flip_direction!(e, g)
        end
        push!(edges, e)
        child = target(e, g)
        push!(treevertices, child)

        # update the frontier
        filter!(x -> x ∉ in_edges(child, g), frontier)
        filter!(x -> x ∉ out_edges(child, g), frontier)
        append!(frontier, x for x in out_edges(child, g) if target(x, g) ∉ treevertices)
        append!(frontier, x for x in in_edges(child, g) if source(x, g) ∉ treevertices)
    end
    length(treevertices) == num_vertices(g) || error("Graph is not connected.")
    SpanningTree(g, root, edges)
end

# adds an edge and vertex to both the tree and the underlying graph
function add_edge!(tree::SpanningTree{V, E}, source::V, target::V, edge::E) where {V, E}
    @assert target ∉ vertices(tree)
    add_edge!(tree.graph, source, target, edge)

    push!(tree.edges, edge)
    push!(tree.inedges, edge)
    push!(tree.outedges, E[])
    push!(out_edges(source, tree), edge)
    resize!(tree.edge_tree_indices, max(edge_index(edge), length(tree.edge_tree_indices)))
    tree.edge_tree_indices[edge_index(edge)] = num_edges(tree)
    tree
end

function replace_edge!(tree::SpanningTree{V, E}, old_edge::E, new_edge::E) where {V, E}
    @assert old_edge ∈ edges(tree)
    src = source(old_edge, tree)
    dest = target(old_edge, tree)
    tree.edges[tree_index(old_edge, tree)] = new_edge
    tree.inedges[vertex_index(dest)] = new_edge
    out_edge_index = findfirst(out_edges(src, tree), old_edge)
    out_edges(src, tree)[out_edge_index] = new_edge
    replace_edge!(tree.graph, old_edge, new_edge)
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

function ancestors(vertex::V, tree::SpanningTree{V, E}) where {V, E}
    ret = [vertex]
    while vertex != root(tree)
        vertex = source(edge_to_parent(vertex, tree), tree)
        push!(ret, vertex)
    end
    ret
end

function lowest_common_ancestor(v1::V, v2::V, tree::SpanningTree{V, E}) where {V, E}
    while v1 != v2
        if tree_index(v1, tree) > tree_index(v2, tree)
            v1 = source(edge_to_parent(v1, tree), tree)
        else
            v2 = source(edge_to_parent(v2, tree), tree)
        end
    end
    v1
end


# TreePath
struct TreePath{V, E}
    source::V
    target::V
    edges::Vector{E} # in order
    directions::UnsafeFastDict{edge_index, E, Symbol} # Symbol: :up if going from source to LCA, :down if going from LCA to target
    indices::IntSet
end

source(path::TreePath) = path.source
target(path::TreePath) = path.target

direction(edge, path::TreePath) = path.directions[edge]

Base.start(path::TreePath) = start(path.edges)
Base.next(path::TreePath, state) = next(path.edges, state)
Base.done(path::TreePath, state) = done(path.edges, state)
Base.eltype(path::TreePath) = eltype(path.edges)
Base.length(path::TreePath) = length(path.edges)
Base.size(path::TreePath) = size(path.edges)
Base.last(path::TreePath) = last(path.edges)

function Base.show(io::IO, path::TreePath)
    println(io, "Path from $(path.source) to $(path.target):")
    for edge in path
        directionchar = ifelse(direction(edge, path) == :up, '↑', '↓')
        print(io, "$directionchar ")
        showcompact(io, edge)
        println(io)
    end
end

function TreePath(src::V, target::V, tree::SpanningTree{V, E}) where {V, E}
    source_to_lca = E[]
    lca_to_target = E[]
    source_current = src
    target_current = target
    while source_current != target_current
        if tree_index(source_current, tree) > tree_index(target_current, tree)
            edge = edge_to_parent(source_current, tree)
            push!(source_to_lca, edge)
            source_current = source(edge, tree)
        else
            edge = edge_to_parent(target_current, tree)
            push!(lca_to_target, edge)
            target_current = source(edge, tree)
        end
    end
    reverse!(lca_to_target)

    edges = collect(flatten((source_to_lca, lca_to_target)))
    directions = UnsafeFastDict{edge_index}(flatten(((e => :up for e in source_to_lca), (e => :down for e in lca_to_target))))
    indices = IntSet(edge_index.(edges))
    TreePath(src, target, edges, directions, indices)
end

Base.@deprecate path(src::V, target::V, tree::SpanningTree{V, E}) where {V, E} TreePath(src, target, tree)

end # module
