import RigidBodyDynamics: Tree, insert!, toposort, leaves, ancestors, path, subtree, reroot

tree = v1 = Tree{Int64, Int32}(1);
v2 = insert!(tree, 2, Int32(2), 1)
v3 = insert!(tree, 3, Int32(3), 2)
v4 = insert!(tree, 4, Int32(4), 1)
v5 = insert!(tree, 5, Int32(5), 4)
v6 = insert!(tree, 6, Int32(6), 4)
v7 = insert!(tree, 7, Int32(7), 5)
v8 = insert!(tree, 8, Int32(8), 7)

@testset "toposort" begin
    toposortedTree = toposort(tree)
    for (index, vertex) in enumerate(toposortedTree)
        if isroot(vertex)
            @test index == 1
        else
            @test index > findfirst(toposortedTree, vertex.parent)
        end
    end
end

@testset "subtree" begin
    sub = subtree(v4)
    @test isroot(sub)
    @test length(toposort(sub)) == 5
    # println(sub)
end

@testset "reroot" begin
    oldRoot = tree
    newRoot = rerooted = reroot(v7, -)
    @test rerooted.vertexData == v7.vertexData
    @test length(toposort(rerooted)) == length(toposort(tree))
    for vertex in toposort(rerooted)
        @test all(path(newRoot, vertex).directions .== 1)
    end

    # ensure that edgeDirectionChangeFunction was applied correctly
    oldAncestors = ancestors(v7)
    for oldTreeVertex in oldAncestors
        newTreeVertex = findfirst(rerooted, oldTreeVertex.vertexData)
        @test all(path(newRoot, newTreeVertex).edgeData .< 0)
    end

    oldNonAncestors = setdiff(toposort(tree), oldAncestors)
    for oldNonAncestor in oldNonAncestors
        newTreeVertex = findfirst(rerooted, oldNonAncestor.vertexData)
        if !isroot(newTreeVertex)
            @test newTreeVertex.edgeToParentData > 0
        end
    end
end
