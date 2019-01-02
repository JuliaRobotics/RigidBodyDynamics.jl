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
tree_index(vertex::V, tree::SpanningTree{V, E}) where {V, E} = vertex === root(tree) ? 1 : tree_index(edge_to_parent(vertex, tree), tree) + 1

function SpanningTree(g::DirectedGraph{V, E}, root::V, edges::AbstractVector{E}) where {V, E}
    n = num_vertices(g)
    length(edges) == n - 1 || error("Expected n - 1 edges.")
    inedges = Vector{E}(undef, n)
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

function SpanningTree(g::DirectedGraph{V, E}, root::V, flipped_edge_map::Union{AbstractDict, Nothing} = nothing;
        next_edge = first #= breadth first =#) where {V, E}
    tree_edges = E[]
    tree_vertices = [root]
    frontier = E[]
    append!(frontier, out_edges(root, g))
    append!(frontier, in_edges(root, g))
    while !isempty(frontier)
        # select a new edge
        e = next_edge(frontier)

        # remove edges from frontier
        flip = source(e, g) ∉ tree_vertices
        child = flip ? source(e, g) : target(e, g)
        filter!(x -> x ∉ in_edges(child, g), frontier)
        filter!(x -> x ∉ out_edges(child, g), frontier)

        # flip current edge if necessary
        if flip
            rewire!(g, e, target(e, g), source(e, g))
            newedge = flip_direction(e)
            replace_edge!(g, e, newedge)
            if flipped_edge_map !== nothing
                flipped_edge_map[e] = newedge
            end
            e = newedge
        end

        # update tree
        push!(tree_edges, e)
        push!(tree_vertices, child)

        # add new edges to frontier
        append!(frontier, x for x in out_edges(child, g) if target(x, g) ∉ tree_vertices)
        append!(frontier, x for x in in_edges(child, g) if source(x, g) ∉ tree_vertices)
    end
    length(tree_vertices) == num_vertices(g) || error("Graph is not connected.")
    SpanningTree(g, root, tree_edges)
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
    out_edge_index = findfirst(isequal(old_edge), out_edges(src, tree))
    out_edges(src, tree)[out_edge_index] = new_edge
    replace_edge!(tree.graph, old_edge, new_edge)
    tree
end

function Base.show(io::IO, tree::SpanningTree, vertex = root(tree), level::Int64 = 0)
    for i = 1 : level print(io, "  ") end
    print(io, "Vertex: ")
    show(IOContext(io, :compact => true), vertex)
    if vertex == root(tree)
        print(io, " (root)")
    else
        print(io, ", ")
        print(io, "Edge: ")
        show(IOContext(io, :compact => true), edge_to_parent(vertex, tree))
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
