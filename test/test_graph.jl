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

        vertex = rand(filter(v -> isempty(in_edges(v, graph)) && isempty(out_edges(v, graph)), vertices(graph)))
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
end
