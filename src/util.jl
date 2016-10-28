# associative type that signifies an empty dictionary and does not allocate memory
type NullDict{K, V} <: Associative{K, V}
end

import Base: length, start, done, get, haskey
length(::NullDict) = 0
start(::NullDict) = 0
done(::NullDict, state) = true
get(::NullDict, key, default) = default
haskey(::NullDict, k) = false

# type of a view of a vector
# TODO: a bit too specific
typealias VectorSegment{T} SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true}

const module_tempdir = joinpath(Base.tempdir(), string(module_name(current_module())))

function cached_download(url::String, localFileName::String, cacheDir::String = joinpath(module_tempdir, string(hash(url))))
    if !ispath(cacheDir)
        mkpath(cacheDir)
    end
    fullCachePath = joinpath(cacheDir, localFileName)
    if !isfile(fullCachePath)
        download(url, fullCachePath)
    end
    fullCachePath
end

macro rtti_dispatch(typeUnion, signature)
    @assert signature.head == :call
    @assert length(signature.args) > 1
    @assert typeUnion.head == :curly
    @assert typeUnion.args[1] == :Union

    f = signature.args[1]
    args = signature.args[2 : end]
    dispatchArg = args[1]
    otherArgs = args[2 : end]
    types = typeUnion.args[2 : end]

    ret = :(error("type not recognized"))
    for T in reverse(types)
        ret = Expr(:if, :(isa($dispatchArg, $T)), :(return $(f)($(dispatchArg)::$T, $(otherArgs...))), ret)
    end
    :($(esc(ret)))
end

typealias ContiguousSMatrixColumnView{S1, S2, T, L} SubArray{T,2,SMatrix{S1, S2, T, L},Tuple{Colon,UnitRange{Int64}},true}
