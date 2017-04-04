Graphs.flip_direction!(edge::Edge{Float64}) = (edge.data = -edge.data)

@testset "graphs" begin
    @testset "disconnected" begin
        graph = DirectedGraph{Vertex{Int64}, Edge{Float64}}()
        verts = [Vertex{Int64}(rand(Int64)) for i = 1 : 10]
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
        show(DevNull, graph)
    end


    @testset "tree graph" begin
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
        show(DevNull, graph)
    end

    @testset "remove_vertex!" begin
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
            v_orig = vertices(original)[findfirst(v_orig -> data(v) == data(v_orig), vertices(original))]
            for (e, e_orig) in zip(in_edges(v, graph), in_edges(v_orig, original))
                @test data(e) == data(e_orig)
            end
            for (e, e_orig) in zip(out_edges(v, graph), out_edges(v_orig, original))
                @test data(e) == data(e_orig)
            end
        end
    end

    @testset "remove_edge!" begin
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
            e_orig = edges(original)[findfirst(e_orig -> data(e) == data(e_orig), edges(original))]
            @test data(source(e, graph)) == data(source(e_orig, original))
            @test data(target(e, graph)) == data(target(e_orig, original))
        end
    end

    @testset "rewire! / flip_direction!" begin
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

        @test data.(vertices(original)) == data.(vertices(graph))
        for e in filter(e -> e != edge, edges(graph))
            e_orig = edges(original)[findfirst(e_orig -> data(e) == data(e_orig), edges(original))]
            @test data(source(e, graph)) == data(source(e_orig, original))
            @test data(target(e, graph)) == data(target(e_orig, original))
        end

        olddata = data(edge)
        flip_direction!(edge, graph)
        @test source(edge, graph) == newtarget
        @test target(edge, graph) == newsource
        @test data(edge) == -olddata
    end

    @testset "SpanningTree" begin
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

        @test all(data.(vertices(tree1)) == data.(vertices(tree2)))
        for (v1, v2) in zip(vertices(tree1), vertices(tree2))
            if v1 == root(tree1)
                @test v2 == root(tree2)
            else
                @test data(edge_to_parent(v1, tree1)) == data(edge_to_parent(v2, tree2))

                outedgedata1 = data.(collect(edges_to_children(v1, tree1)))
                outedgedata2 = data.(collect(edges_to_children(v2, tree2)))
                @test isempty(setdiff(outedgedata1, outedgedata2))

                @test isempty(setdiff(out_edges(v1, graph1), edges_to_children(v1, tree1)))
                @test isempty(setdiff(out_edges(v2, graph2), edges_to_children(v2, tree2)))
            end
        end

        tree = tree1
        show(DevNull, tree)

        @test_throws AssertionError add_edge!(tree, rand(vertices(tree)), rand(vertices(tree)), Edge(rand(Int32)))

        for src in vertices(tree)
            for dest in vertices(tree)
                src_ancestors = ancestors(src, tree)
                dest_ancestors = ancestors(dest, tree)
                lca = lowest_common_ancestor(src, dest, tree)
                p = path(src, dest, tree)
                @test source(p) == src
                @test target(p) == dest

                for (v, v_ancestors, pathsegment) in [(src, src_ancestors, p.source_to_lca); (dest, dest_ancestors, p.target_to_lca)]
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
                        @test v ∉ (v -> source(v, tree)).(p.source_to_lca)
                        @test v ∉ (v -> source(v, tree)).(p.target_to_lca)
                    end
                end
                for v in setdiff(src_ancestors, dest_ancestors)
                    @test tree_index(v, tree) > tree_index(lca, tree)
                end
            end
        end
    end
end
