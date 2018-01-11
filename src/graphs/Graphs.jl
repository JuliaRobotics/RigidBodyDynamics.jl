module Graphs

# types
export
    DirectedGraph,
    SpanningTree,
    TreePath,
    Edge,
    Vertex

# functions
export
    data,
    vertices,
    edges,
    source,
    target,
    out_edges,
    in_edges,
    out_neighbors,
    in_neighbors,
    num_vertices,
    num_edges,
    add_vertex!,
    add_edge!,
    remove_vertex!,
    remove_edge!,
    rewire!,
    replace_edge!,
    reindex!,
    root,
    edge_to_parent,
    edges_to_children,
    tree_index,
    ancestors,
    lowest_common_ancestor,
    direction

using RigidBodyDynamics.CustomCollections: UnsafeFastDict
using Base.Iterators: flatten

include("abstract.jl")
include("directed_graph.jl")
include("spanning_tree.jl")
include("tree_path.jl")
include("edge_vertex_wrappers.jl")

end # module
