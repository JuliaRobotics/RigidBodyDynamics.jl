module TreeDataStructure

import Base: showcompact, show, parent, findfirst, map!, insert!

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
    ancestors,
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

    TreeVertex(vertexData::V) = new(vertexData, [], Nullable{Pair{TreeVertex{V, E}, E}}())
    TreeVertex{V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E) = new(vertexData, [], parent => edgeData)
end
TreeVertex{V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E) = TreeVertex{V, E}(vertexData, parent, edgeData)

typealias Tree{V, E} TreeVertex{V, E}

vertex_data(v::TreeVertex) = v.vertexData
children(v::TreeVertex) = v.children
parent(v::TreeVertex) = get(v.parentAndEdgeData)[1]
edge_to_parent_data(v::TreeVertex) = get(v.parentAndEdgeData)[2]

isroot{V, E}(v::TreeVertex{V, E}) = isnull(v.parentAndEdgeData)
isleaf{V, E}(v::TreeVertex{V, E}) = isempty(children(v))

function showcompact(io::IO, vertex::TreeVertex)
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

function show(io::IO, vertex::TreeVertex, level = 0)
    for i = 1 : level print(io, "  ") end
    showcompact(io, vertex)
    for child in children(vertex)
        print(io, "\n")
        show(io, child, level + 1)
    end
end

# TODO: not type stable
function findfirst{V, E}(pred::Function, tree::Tree{V, E})
    # depth first search
    root = tree
    pred(root) && return root

    for child in children(root)
        vertex = findfirst(pred, child)
        vertex != nothing && return vertex
    end
    return nothing
end

# TODO: not type stable
function findfirst{V, E}(tree::Tree{V, E}, vertexData::V)
    # depth first search
    root = tree
    vertex_data(root) == vertexData && return root

    for child in children(root)
        vertex = findfirst(child, vertexData)
        vertex != nothing && return vertex
    end
    return nothing
end

function find{V, E}(pred::Function, tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    pred(root) && push!(result, root)
    for child in children(root)
        find(pred, child, result)
    end
    return result
end

function toposort{V, E}(tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    push!(result, root)
    for child in children(root)
        toposort(child, result)
    end
    return result
end

function insert!{V, E}(parentVertex::TreeVertex{V, E}, childVertex::TreeVertex{V, E}, edgeData::E = edge_to_parent_data(childVertex))
    # Note: removes any previously existing parent/child relationship for childVertex
    if !isroot(childVertex)
        parentsChildren = children(parent(vertex))
        deleteat!(parentsChildren, findfirst(parentsChildren, vertex))
    end
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

function ancestors{V, E}(vertex::TreeVertex{V, E})
    current = vertex
    result = Vector{TreeVertex{V, E}}()
    while !isroot(current)
        push!(result, parent(current))
        current = parent(current)
    end
    return result
end

function leaves{V, E}(tree::Tree{V, E})
    return find(x -> isleaf(x), tree)
end

function map!{F, V, E}(f::F, tree::Tree{V, E})
    root = tree
    f(root)
    for child in children(root)
        map!(f, child)
    end
    return tree
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

function show(io::IO, p::Path)
    println(io, "Path:")
    println(io, "Vertices: $(vertex_data(p))")
    println(io, "Edges: $(p.edgeData)")
    print(io, "Directions: $(p.directions)")
end

function path{V, E}(from::TreeVertex{V, E}, to::TreeVertex{V, E})
    ancestorsFrom = [from; ancestors(from)]
    ancestorsTo = [to; ancestors(to)]

    # find least common ancestor
    common_size = min(length(ancestorsFrom), length(ancestorsTo))
    lca_found = false
    fromIndex = length(ancestorsFrom) - common_size + 1
    toIndex = length(ancestorsTo) - common_size + 1
    i = 1
    while i <= common_size
        if ancestorsFrom[fromIndex] == ancestorsTo[toIndex]
            lca_found = true
            break
        end
        i += 1
        fromIndex += 1
        toIndex += 1
    end
    !lca_found && error("no path between vertices")

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
    return Path(vertexData, edgeData, directions)
end

end # module TreeDataStructure
