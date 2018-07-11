# Vertex and Edge types; useful for wrapping an existing type with the edge interface.
# Note that DirectedGraph does not require using these types; just implement the edge interface.
for typename in (:Edge, :Vertex)
    getid = Symbol(lowercase(string(typename)) * "_id")
    setid = Symbol("set_" * lowercase(string(typename)) * "_id!")
    @eval begin
        mutable struct $typename{T}
            data::T
            id::Int64
        end
        $typename(data) = $typename(data, -1)
        $getid(x::$typename) = x.id
        $setid(x::$typename, index::Int64) = (x.id = index)
    end
end
