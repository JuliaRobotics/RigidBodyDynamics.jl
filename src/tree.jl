type TreeVertex{V, E}
    vertexData::V
    children::Vector{TreeVertex{V, E}}
    parent::TreeVertex{V, E}
    edgeToParentData::E

    TreeVertex{V}(vertexData::V) = new(vertexData, [])
    TreeVertex{V, E}(vertexData::V, parent::TreeVertex{V, E}, edgeData::E) = new(vertexData, [], parent, edgeData)
end
typealias Tree{V, E} TreeVertex{V, E}

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

function ancestors{V, E}(vertex::TreeVertex{V, E}, result = Vector{TreeVertex{V, E}}())
    # includes self
    push!(result, vertex)
    !isroot(vertex) && ancestors(vertex.parent, result)
    return result
end

function least_common_ancestor{V, E}(v1::TreeVertex{V, E}, v2::TreeVertex{V, E})
    ancestors1 = ancestors(v1);
    ancestors2 = ancestors(v2);
    for i = min(length(ancestors1), length(ancestors2)) : -1 : 2
        ancestors1[i] != ancestors2[i] && return ancestors1[i + 1]
    end
    return ancestors1[1]
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
