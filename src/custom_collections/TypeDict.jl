struct TypeDict{C, V}
    value_constructor::C
    value_type_fun::V
    keys::Vector{UInt}
    values::Vector
end

function TypeDict(value_constructor::C, value_type_fun::V) where {C, V}
    TypeDict{C, V}(value_constructor, value_type_fun, UInt[], [])
end

Base.show(io::IO, ::TypeDict) = print(io, "TypeDict{…}(…)")

function Base.getindex(dict::D, ::Type{T}) where {D<:TypeDict, T}
    ReturnType = dict.value_type_fun(T)
    key = objectid(T)
    @inbounds for i in eachindex(dict.keys)
        if dict.keys[i] === key
            return dict.values[i]::ReturnType
        end
    end
    value = dict.value_constructor(T)::ReturnType
    push!(dict.keys, key)
    push!(dict.values, value)
    return value::ReturnType
end
