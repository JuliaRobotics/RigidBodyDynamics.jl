Graphs.flip_direction(edge::Edge{Int32}) = Edge(-edge.data)

@testset "graphs" begin
    @testset "disconnected" begin
        Random.seed!(8)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        verts = [Vertex(rand(Int64)) for i = 1 : 10]
        for v in verts
            add_vertex!(graph, v)
        end
        @test num_vertices(graph) == length(verts)
        @test num_edges(graph) == 0
        for v in verts
            @test length(in_edges(v, graph)) == 0
            @test length(out_edges(v, graph)) == 0
            @test length(in_neighbors(v, graph)) == 0
            @test length(out_neighbors(v, graph)) == 0
        end
        @test isempty(setdiff(vertices(graph), verts))
        @test isempty(edges(graph))
        show(devnull, graph)
    end


    @testset "tree graph" begin
        Random.seed!(9)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        root = Vertex(rand(Int64))
        add_vertex!(graph, root)
        nedges = 15
        for i = 1 : nedges
            parent = rand(vertices(graph))
            child = Vertex(rand(Int64))
            edge = Edge(rand())
            add_edge!(graph, parent, child, edge)
        end
        @test num_vertices(graph) == nedges + 1
        @test num_edges(graph) == nedges
        for v in vertices(graph)
            if v == root
                @test length(in_edges(v, graph)) == 0
                @test length(out_edges(v, graph)) > 0
                @test length(in_neighbors(v, graph)) == 0
                @test length(out_neighbors(v, graph)) > 0
            else
                @test length(in_edges(v, graph)) == 1
                @test length(in_neighbors(v, graph)) == 1
            end
        end
        show(devnull, graph)
    end

    @testset "remove_vertex!" begin
        Random.seed!(10)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        edge = Edge(rand())
        add_edge!(graph, Vertex(rand(Int64)), Vertex(rand(Int64)), edge)
        for v in vertices(graph)
            @test_throws ErrorException remove_vertex!(graph, v)
        end

        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        for i = 1 : 100
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float64(i)))
        end
        original = deepcopy(graph)

        vertex = rand(collect(filter(v -> isempty(in_edges(v, graph)) && isempty(out_edges(v, graph)), vertices(graph))))
        remove_vertex!(graph, vertex)
        @test vertex ∉ vertices(graph)
        for v in vertices(graph)
            v_orig = vertices(original)[findfirst(v_orig -> v.data == v_orig.data, vertices(original))]
            for (e, e_orig) in zip(in_edges(v, graph), in_edges(v_orig, original))
                @test e.data == e_orig.data
            end
            for (e, e_orig) in zip(out_edges(v, graph), out_edges(v_orig, original))
                @test e.data == e_orig.data
            end
        end
    end

    @testset "remove_edge!" begin
        Random.seed!(11)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        for i = 1 : 100
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float64(i)))
        end
        original = deepcopy(graph)

        edge = rand(edges(graph))
        remove_edge!(graph, edge)
        @test edge ∉ edges(graph)
        @test num_edges(graph) == num_edges(original) - 1
        for v in vertices(graph)
            @test edge ∉ in_edges(v, graph)
            @test edge ∉ out_edges(v, graph)
        end

        for e in edges(graph)
            e_orig = edges(original)[findfirst(e_orig -> e.data == e_orig.data, edges(original))]
            @test source(e, graph).data == source(e_orig, original).data
            @test target(e, graph).data == target(e_orig, original).data
        end
    end

    @testset "rewire!" begin
        Random.seed!(12)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        for i = 1 : 100
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float64(i)))
        end
        original = deepcopy(graph)

        edge = rand(edges(graph))
        oldsource = source(edge, graph)
        oldtarget = target(edge, graph)
        non_source_vertices = delete!(Set(vertices(graph)), oldsource)
        non_target_vertices = delete!(Set(vertices(graph)), oldtarget)
        newsource = rand(collect(non_source_vertices))
        newtarget = rand(collect(non_target_vertices))

        rewire!(graph, edge, newsource, newtarget)
        @test source(edge, graph) == newsource
        @test target(edge, graph) == newtarget
        @test edge ∈ out_edges(newsource, graph)
        @test edge ∈ in_edges(newtarget, graph)
        @test edge ∉ out_edges(oldsource, graph)
        @test edge ∉ in_edges(oldtarget, graph)

        @test map(x -> x.data, vertices(original)) == map(x -> x.data, vertices(graph))
        for e in filter(e -> e != edge, edges(graph))
            e_orig = edges(original)[findfirst(e_orig -> e.data == e_orig.data, edges(original))]
            @test source(e, graph).data == source(e_orig, original).data
            @test target(e, graph).data == target(e_orig, original).data
        end
    end

    @testset "replace_edge!" begin
        Random.seed!(13)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        for i = 1 : 100
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float64(i)))
        end
        original = deepcopy(graph)

        for i = 1 : 10
            old_edge = rand(edges(graph))
            src = source(old_edge, graph)
            dest = target(old_edge, graph)
            new_edge = Edge(NaN)
            replace_edge!(graph, old_edge, new_edge)

            @test Graphs.edge_index(old_edge) == -1
            @test all(map(x -> x.data, vertices(graph)) .== map(x -> x.data, vertices(original)))
            @test new_edge ∈ in_edges(dest, graph)
            @test old_edge ∉ in_edges(dest, graph)
            @test new_edge ∈ out_edges(src, graph)
            @test old_edge ∉ out_edges(src, graph)
            @test source(new_edge, graph) == src
            @test target(new_edge, graph) == dest
            @test isnan(new_edge.data)
        end
    end

    @testset "SpanningTree" begin
        Random.seed!(14)
        rootdata = 0

        # graph1: tree grown incrementally
        graph1 = DirectedGraph{Vertex{Int64}, Edge{Int32}}()
        root1 = Vertex(rootdata)
        add_vertex!(graph1, root1)
        tree1 = SpanningTree(graph1, root1)

        # graph2: tree constructed after graph is built
        graph2 = DirectedGraph{Vertex{Int64}, Edge{Int32}}()
        root2 = Vertex(rootdata)
        add_vertex!(graph2, root2)

        nedges = 15
        for i = 1 : nedges
            parentind = rand(1 : num_vertices(graph1))
            childdata = i
            edgedata = Int32(i + 3)
            add_edge!(tree1, vertices(tree1)[parentind], Vertex(childdata), Edge(edgedata))
            add_edge!(graph2, vertices(graph2)[parentind], Vertex(childdata), Edge(edgedata))
        end

        tree2 = SpanningTree(graph2, root2)

        @test all(map(x -> x.data, vertices(tree1)) == map(x -> x.data, vertices(tree2)))
        for (v1, v2) in zip(vertices(tree1), vertices(tree2))
            if v1 == root(tree1)
                @test v2 == root(tree2)
            else
                @test edge_to_parent(v1, tree1).data == edge_to_parent(v2, tree2).data

                outedgedata1 = map(x -> x.data, collect(edges_to_children(v1, tree1)))
                outedgedata2 = map(x -> x.data, collect(edges_to_children(v2, tree2)))
                @test isempty(setdiff(outedgedata1, outedgedata2))

                @test isempty(setdiff(out_edges(v1, graph1), edges_to_children(v1, tree1)))
                @test isempty(setdiff(out_edges(v2, graph2), edges_to_children(v2, tree2)))
            end
        end

        tree = tree1
        show(devnull, tree)

        @test_throws AssertionError add_edge!(tree, rand(vertices(tree)), rand(vertices(tree)), Edge(rand(Int32)))

        for src in vertices(tree)
            for dest in vertices(tree)
                src_ancestors = ancestors(src, tree)
                dest_ancestors = ancestors(dest, tree)
                lca = lowest_common_ancestor(src, dest, tree)
                p = TreePath(src, dest, tree)

                show(devnull, p)
                @inferred collect(p)

                @test source(p) == src
                @test target(p) == dest

                source_to_lca = collect(edge for edge in p if direction(edge, p) == PathDirections.up)
                target_to_lca = reverse!(collect(edge for edge in p if direction(edge, p) == PathDirections.down))

                for (v, v_ancestors, pathsegment) in [(src, src_ancestors, source_to_lca); (dest, dest_ancestors, target_to_lca)]
                    if v == root(tree)
                        @test tree_index(v, tree) == 1
                        @test lca == v
                        @test length(v_ancestors) == 1
                        @test isempty(pathsegment)
                    end
                    for v_ancestor in v_ancestors
                        @test tree_index(v_ancestor, tree) <= tree_index(v, tree)
                    end
                    @test lca ∈ v_ancestors
                    if v != lca
                        @test source(last(pathsegment), tree) == lca
                    end
                end

                for v in intersect(src_ancestors, dest_ancestors)
                    @test tree_index(v, tree) <= tree_index(lca, tree)
                    if v != lca
                        @test v ∉ (v -> source(v, tree)).(source_to_lca)
                        @test v ∉ (v -> source(v, tree)).(target_to_lca)
                    end
                end
                for v in setdiff(src_ancestors, dest_ancestors)
                    @test tree_index(v, tree) > tree_index(lca, tree)
                end
            end
        end

        for i = 1 : 10
            old_edge = rand(edges(tree))

            src = source(old_edge, tree)
            dest = target(old_edge, tree)
            old_id = Graphs.edge_index(old_edge)

            # make sure that replacing edge with itself doesn't mess with anything
            replace_edge!(tree, old_edge, old_edge)
            @test source(old_edge, tree) == src
            @test target(old_edge, tree) == dest
            @test Graphs.edge_index(old_edge) == old_id

            # replace with a new edge
            d = Int32(-10 * i)
            new_edge = Edge(d)
            replace_edge!(tree, old_edge, new_edge)

            @test Graphs.edge_index(old_edge) == -1
            @test new_edge ∈ in_edges(dest, tree)
            @test old_edge ∉ in_edges(dest, tree)
            @test new_edge ∈ out_edges(src, tree)
            @test old_edge ∉ out_edges(src, tree)
            @test source(new_edge, tree) == src
            @test target(new_edge, tree) == dest
            @test new_edge.data == d
            @test edge_to_parent(dest, tree) == new_edge
            @test new_edge ∈ edges_to_children(src, tree)
        end

        original = deepcopy(graph2)
        edgemap = Dict(zip(edges(original), edges(graph2)))
        newroot = rand(setdiff(vertices(graph2), [root2]))
        flipped_edge_map = Dict{Edge{Int32}, Edge{Int32}}()
        newtree = SpanningTree(graph2, newroot, flipped_edge_map)

        @test !isempty(flipped_edge_map)
        for (oldedge, newedge) in edgemap
            flipped = haskey(flipped_edge_map, newedge)
            if flipped
                newedge = flipped_edge_map[newedge]
            end

            old_source_ind = Graphs.vertex_index(source(oldedge, original))
            old_target_ind = Graphs.vertex_index(target(oldedge, original))
            new_source_ind = Graphs.vertex_index(source(newedge, graph2))
            new_target_ind = Graphs.vertex_index(target(newedge, graph2))

            if flipped
                @test oldedge.data == -newedge.data
                @test new_source_ind == old_target_ind
                @test new_target_ind == old_source_ind
            else
                @test oldedge.data == newedge.data
                @test new_source_ind == old_source_ind
                @test new_target_ind == old_target_ind
            end
        end
    end

    @testset "reindex!" begin
        Random.seed!(15)
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        for i = 1 : 100
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float64(i)))
        end
        newvertices = shuffle(vertices(graph))
        newedges = shuffle(edges(graph))
        reindex!(graph, newvertices, newedges)
        @test all(vertices(graph) .== newvertices)
        @test all(edges(graph) .== newedges)
        @test all(Graphs.vertex_index.(vertices(graph)) .== 1 : num_vertices(graph))
        @test all(Graphs.edge_index.(edges(graph)) .== 1 : num_edges(graph))
    end

    @testset "map-like constructor" begin
        Random.seed!(16)
        graph = DirectedGraph{Vertex{Int32}, Edge{Float32}}()
        for i = Int32(1) : Int32(100)
            add_vertex!(graph, Vertex(i))
        end
        for i = 1 : num_vertices(graph) - 1
            add_edge!(graph, rand(vertices(graph)), rand(vertices(graph)), Edge(Float32(i)))
        end
        mappedgraph = DirectedGraph(x -> Vertex(Int64(x.data), x.id), x -> Edge(Float64(x.data), x.id), graph)

        @test vertextype(mappedgraph) == Vertex{Int64}
        @test edgetype(mappedgraph) == Edge{Float64}
        @test all(v1.data == v2.data for (v1, v2) in zip(vertices(graph), vertices(mappedgraph)))
        @test all(e1.data == e2.data for (e1, e2) in zip(edges(graph), edges(mappedgraph)))
        @test all(source(e1, graph).data == source(e2, mappedgraph).data for (e1, e2) in zip(edges(graph), edges(mappedgraph)))
        @test all(target(e1, graph).data == target(e2, mappedgraph).data for (e1, e2) in zip(edges(graph), edges(mappedgraph)))

        inedgesmatch(v1, v2) = all(e1.data == e2.data for (e1, e2) in zip(in_edges(v1, graph), in_edges(v2, mappedgraph)))
        @test all(inedgesmatch(v1, v2) for (v1, v2) in zip(vertices(graph), vertices(mappedgraph)))

        outedgesmatch(v1, v2) = all(e1.data == e2.data for (e1, e2) in zip(out_edges(v1, graph), out_edges(v2, mappedgraph)))
        @test all(outedgesmatch(v1, v2) for (v1, v2) in zip(vertices(graph), vertices(mappedgraph)))
    end
end
