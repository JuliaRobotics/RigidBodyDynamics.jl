module PathDirections
export PathDirection
@enum PathDirection::Bool begin
    up # going towards the root of the tree
    down # going away from the root of the tree
end
end
using .PathDirections

struct TreePath{V, E}
    source::V
    target::V
    edges::Vector{E} # in order
    directions::Vector{PathDirection}
    indexmap::Vector{Int}
end

source(path::TreePath) = path.source
target(path::TreePath) = path.target

@inline Base.findfirst(path::TreePath{<:Any, E}, edge::E) where {E} = path.indexmap[edge_index(edge)] # TODO: remove, use equalto
@inline Base.in(edge::E, path::TreePath{<:Any, E}) where {E} = findfirst(path, edge) != 0 # TODO: findfirst update
@inline directions(path::TreePath) = path.directions
@inline direction(edge::E, path::TreePath{<:Any, E}) where {E} = path.directions[findfirst(path, edge)] # TODO: findfirst update

Base.iterate(path::TreePath) = iterate(path.edges)
Base.iterate(path::TreePath, state) = iterate(path.edges, state)
Base.eltype(path::TreePath) = eltype(path.edges)
Base.length(path::TreePath) = length(path.edges)

function Base.show(io::IO, path::TreePath)
    println(io, "Path from $(path.source) to $(path.target):")
    for edge in path
        directionchar = ifelse(direction(edge, path) == PathDirections.up, '↑', '↓')
        print(io, "$directionchar ")
        show(IOContext(io, :compact => true), edge)
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

    edges = collect(E, flatten((source_to_lca, lca_to_target)))
    directions = collect(PathDirection, flatten(((PathDirections.up for e in source_to_lca), (PathDirections.down for e in lca_to_target))))
    indexmap = Vector(sparsevec(Dict(edge_index(e) => i for (i, e) in enumerate(edges)), num_edges(tree)))
    TreePath{V, E}(src, target, edges, directions, indexmap)
end
