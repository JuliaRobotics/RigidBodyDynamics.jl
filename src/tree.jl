module TreeDataStructure

using Compat

import Base: parent, findfirst, map!, insert!, copy

import Compat.Iterators: filter

export
    # types
    Tree,
    TreeVertex,
    Path,

    # functions
    isroot,
    isleaf,
    toposort,
    detach!,
    isancestor,
    ancestors,
    lowest_common_ancestor,
    leaves,
    path,
    insert_subtree!,
    subtree,
    reroot,
    merge_into_parent!,
    vertex_data,
    children,
    edge_to_parent_data

type TreeVertex{V, E}
    vertexData::V
    children::Vector{TreeVertex{V, E}}
    parentAndEdgeData::Nullable{Pair{TreeVertex{V, E}, E}}

    (::Type{TreeVertex{V, E}}){V, E}(vertexData::V) = new{V, E}(vertexData, [], Nullable{Pair{TreeVertex{V, E}, E}}())

    function (::Type{TreeVertex{V, E}}){V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E)
        new{V, E}(vertexData, [], parent => edgeData)
    end
end
TreeVertex{V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E) = TreeVertex{V, E}(vertexData, parent, edgeData)

@compat const Tree{V, E} = TreeVertex{V, E}

vertex_data(v::TreeVertex) = v.vertexData
children(v::TreeVertex) = v.children
parent(v::TreeVertex) = get(v.parentAndEdgeData)[1]
edge_to_parent_data(v::TreeVertex) = get(v.parentAndEdgeData)[2]

isroot{V, E}(v::TreeVertex{V, E}) = isnull(v.parentAndEdgeData)
isleaf{V, E}(v::TreeVertex{V, E}) = isempty(children(v))

function Base.showcompact(io::IO, vertex::TreeVertex)
    print(io, "Vertex: ")
    showcompact(io, vertex_data(vertex))
    if isroot(vertex)
        print(io, " (root)")
    else
        print(io, ", ")
        print(io, "Edge: ")
        showcompact(io, edge_to_parent_data(vertex))
    end
end

function Base.show(io::IO, vertex::TreeVertex, level = 0)
    if get(io, :compact, false)
        showcompact(io, vertex)
    else
        for i = 1 : level print(io, "  ") end
        showcompact(io, vertex)
        for child in children(vertex)
            print(io, "\n")
            show(io, child, level + 1)
        end
    end
end

function _findfirst{V, E}(pred, tree::Tree{V, E})::Nullable{TreeVertex{V, E}}
    # depth first search
    root = tree
    pred(root) && return Nullable(root)

    for child in children(root)
        vertex = _findfirst(pred, child)
        !isnull(vertex) && return vertex
    end
    Nullable{TreeVertex{V, E}}()
end

findfirst(pred::Function, tree::Tree) = get(_findfirst(pred, tree))
findfirst{V, E}(tree::Tree{V, E}, vertexData::V) = findfirst(v -> vertex_data(v) == vertexData, tree)

function find{V, E}(pred::Function, tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    pred(root) && push!(result, root)
    for child in children(root)
        find(pred, child, result)
    end
    result
end

function toposort{V, E}(tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    push!(result, root)
    for child in children(root)
        toposort(child, result)
    end
    result
end

function detach!{V, E}(vertex::TreeVertex{V, E})
    if !isroot(vertex)
        index = findfirst(parent(vertex).children, vertex)
        if index > 0
            deleteat!(parent(vertex).children, index)
        end
    end
    vertex.parentAndEdgeData = Nullable{Pair{TreeVertex{V, E}, E}}()
    vertex
end

function insert!{V, E}(parentVertex::TreeVertex{V, E}, childVertex::TreeVertex{V, E}, edgeData::E = edge_to_parent_data(childVertex))
    # Note: removes any previously existing parent/child relationship for childVertex
    detach!(childVertex)
    childVertex.parentAndEdgeData = parentVertex => edgeData
    push!(children(parentVertex), childVertex)
    childVertex
end

function insert!{V, E}(parentVertex::TreeVertex{V, E}, vertexData::V, edgeData::E)
    vertex = TreeVertex{V, E}(vertexData, parentVertex, edgeData)
    push!(parentVertex.children, vertex)
    vertex
end

function insert!{V, E}(tree::Tree{V, E}, vertexData::V, edgeData::E, parentData::V)
    parentVertex = findfirst(tree, parentData)
    parentVertex == nothing && error("parent not found")
    insert!(parentVertex, vertexData, edgeData)
end

function copy{V, E}(v::TreeVertex{V, E})
    ret = isroot(v) ? TreeVertex{V, E}(vertex_data(v)) : TreeVertex(vertex_data(v), parent(v), edge_to_parent_data(v))
    for child in children(v)
        insert!(ret, copy(child))
    end
    ret
end

function isancestor{V, E}(vertex::TreeVertex{V, E}, candidate::TreeVertex{V, E})
    current = vertex
    while !isroot(current)
        current = parent(current)
        current == candidate && return true
    end
    return false
end

function lowest_common_ancestor{V, E}(a::TreeVertex{V, E}, b::TreeVertex{V, E})
    (a == b || isroot(b)) && return b
    current = a
    while !isancestor(b, current)
        isroot(current) && error("vertices are not part of the same tree")
        current = parent(current)
    end
    current
end

function ancestors{V, E}(vertex::TreeVertex{V, E})
    current = vertex
    result = Vector{TreeVertex{V, E}}()
    while !isroot(current)
        push!(result, parent(current))
        current = parent(current)
    end
    result
end

function leaves{V, E}(tree::Tree{V, E})
    find(x -> isleaf(x), tree)
end

function map!{F, V, E}(f::F, tree::Tree{V, E})
    root = tree
    f(root)
    for child in children(root)
        map!(f, child)
    end
    tree
end

function insert_subtree!{V, E}(root::TreeVertex{V, E}, subtree_root::TreeVertex{V, E})
    # modifies root, but doesn't modify subtree_root
    inserted = insert!(root, vertex_data(subtree_root), edge_to_parent_data(subtree_root))
    for child in children(subtree_root)
        insert_subtree!(inserted, child)
    end
    inserted
end

function subtree{V, E}(vertex::TreeVertex{V, E})
    ret = Tree{V, E}(vertex_data(vertex))
    for child in children(vertex)
        insert_subtree!(ret, child)
    end
    ret
end

function reroot{V, E, F}(newRoot::TreeVertex{V, E}, edgeDirectionChangeFunction::F = identity)
    ret = Tree{V, E}(vertex_data(newRoot))

    currentVertexNewTree = ret
    previousVertexOldTree = newRoot
    currentVertexOldTree = newRoot

    done = false
    while !done
        for child in children(currentVertexOldTree)
            if child != previousVertexOldTree
                insert_subtree!(currentVertexNewTree, child)
            end
        end

        done = isroot(currentVertexOldTree)

        if !done
            vertexData = vertex_data(parent(currentVertexOldTree))
            edgeToParentData = edgeDirectionChangeFunction(edge_to_parent_data(currentVertexOldTree))
            currentVertexNewTree = insert!(currentVertexNewTree, vertexData, edgeToParentData)
            previousVertexOldTree = currentVertexOldTree
            currentVertexOldTree = parent(currentVertexOldTree)
        end
    end
    ret
end

function merge_into_parent!(vertex::TreeVertex)
    # splice vertex's children into parent's children at vertex's location
    @assert !isroot(vertex)
    for child in children(vertex)
        child.parentAndEdgeData = parent(vertex) => edge_to_parent_data(child)
    end
    parentsChildren = parent(vertex).children
    splice!(parentsChildren, findfirst(parentsChildren, vertex), children(vertex))
    detach!(vertex)
    nothing
end

immutable Path{V, E}
    vertexData::Vector{V}
    edgeData::Vector{E}
    directions::Vector{Int64}
end

function Base.show(io::IO, p::Path)
    println(io, "Path:")
    println(io, "Vertices: $(p.vertexData)")
    println(io, "Edges: $(p.edgeData)")
    print(io, "Directions: $(p.directions)")
end

function path{V, E}(from::TreeVertex{V, E}, to::TreeVertex{V, E})
    ancestorsFrom = [from; ancestors(from)]
    ancestorsTo = [to; ancestors(to)]

    # find least common ancestor
    commonSize = min(length(ancestorsFrom), length(ancestorsTo))
    lowestCommonAncestorFound = false
    fromIndex = length(ancestorsFrom) - commonSize + 1
    toIndex = length(ancestorsTo) - commonSize + 1
    i = 1
    while i <= commonSize
        if ancestorsFrom[fromIndex] == ancestorsTo[toIndex]
            lowestCommonAncestorFound = true
            break
        end
        i += 1
        fromIndex += 1
        toIndex += 1
    end
    !lowestCommonAncestorFound && error("no path between vertices")

    vertexData = Vector{V}()
    edgeData = Vector{E}()
    directions = Vector{Int64}()
    for j = 1 : fromIndex - 1
        push!(vertexData, vertex_data(ancestorsFrom[j]))
        push!(edgeData, edge_to_parent_data(ancestorsFrom[j]))
        push!(directions, -1)
    end
    push!(vertexData, vertex_data(ancestorsFrom[fromIndex]))
    for j = toIndex - 1 : -1 : 1
        push!(vertexData, vertex_data(ancestorsTo[j]))
        push!(edgeData, edge_to_parent_data(ancestorsTo[j]))
        push!(directions, 1)
    end
    Path(vertexData, edgeData, directions)
end

end # module TreeDataStructure
