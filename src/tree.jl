type TreeVertex{V, E}
    vertexData::V
    children::Vector{TreeVertex{V, E}}
    parent::TreeVertex{V, E}
    edgeToParentData::E

    TreeVertex{V}(vertexData::V) = new(vertexData, [])
    TreeVertex{V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E) = new(vertexData, [], parent, edgeData)
end
typealias Tree{V, E} TreeVertex{V, E}

immutable Path{V, E}
    vertexData::Vector{V}
    edgeData::Vector{E}
    directions::Vector{Int64}
end

isroot{V, E}(v::TreeVertex{V, E}) = !isdefined(v, :parent)
isleaf{V, E}(v::TreeVertex{V, E}) = isempty(v.children)

function findfirst{V, E}(pred::Function, tree::Tree{V, E})
    # depth first search
    root = tree
    pred(root) && return root

    for child in root.children
        vertex = findfirst(pred, child)
        vertex != nothing && return vertex
    end
    return nothing
end

function findfirst{V, E}(tree::Tree{V, E}, vertexData::V)
    # depth first search
    root = tree
    root.vertexData == vertexData && return root

    for child in root.children
        vertex = findfirst(child, vertexData)
        vertex != nothing && return vertex
    end
    return nothing
end

function find{V, E}(pred::Function, tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    pred(root) && push!(result, root)
    for child in root.children
        find(pred, child, result)
    end
    return result
end

function toposort{V, E}(tree::Tree{V, E}, result = Vector{TreeVertex{V, E}}())
    root = tree
    push!(result, root)
    for child in root.children
        toposort(child, result)
    end
    return result
end

function insert!{V, E}(tree::Tree{V, E}, vertexData::V, edgeData::E, parentData::V)
    parentVertex = findfirst(tree, parentData)
    parentVertex == nothing && error("parent not found")
    vertex = TreeVertex{V, E}(vertexData, parentVertex, edgeData)
    push!(parentVertex.children, vertex)
    return vertex
end

function ancestors{V, E}(vertex::TreeVertex{V, E})
    current = vertex
    result = Vector{TreeVertex{V, E}}()
    while !isroot(current)
        push!(result, current.parent)
        current = current.parent
    end
    return result
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
        push!(vertexData, ancestorsFrom[j].vertexData)
        push!(edgeData, ancestorsFrom[j].edgeToParentData)
        push!(directions, -1)
    end
    push!(vertexData, ancestorsFrom[fromIndex].vertexData)
    for j = toIndex - 1 : -1 : 1
        push!(vertexData, ancestorsTo[j].vertexData)
        push!(edgeData, ancestorsTo[j].edgeToParentData)
        push!(directions, 1)
    end
    return Path(vertexData, edgeData, directions)
end

function leaves{V, E}(tree::Tree{V, E})
    return find(x -> isleaf(x), tree)
end

function map!{F, V, E}(f::F, tree::Tree{V, E})
    root = tree
    f(root)
    for child in root.children
        map!(f, child)
    end
    return tree
end
