# associative type that signifies an empty dictionary and does not allocate memory
immutable NullDict{K, V} <: Associative{K, V}
end
Base.length(::NullDict) = 0
Base.start(::NullDict) = 0
Base.done(::NullDict, state) = true
Base.get(::NullDict, key, default) = default
Base.haskey(::NullDict, k) = false

# ultimate sparse AbstractVector type that does not allocate memory
immutable NullVector{T} <: AbstractVector{T}
    length::Int64
end
Base.size(A::NullVector) = (A.length, )
Base.getindex{T}(A::NullVector{T}, i::Int) = zero(T)
Base.setindex!(A::NullVector, v, i::Int) = error()
Base.setindex!{N}(A::NullVector, v, I::Vararg{Int, N}) = error()
Base.linearindexing(::Type{NullVector}) = Base.LinearFast()

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

if VERSION < v"0.6-" # TODO: 0.5 hack: broadcast! allocates in 0.5; fixed in 0.6.
    function sub!(out, a, b)
        @boundscheck length(out) == length(a) || error("size mismatch")
        @boundscheck length(out) == length(b) || error("size mismatch")
        @simd for i in eachindex(out)
            @inbounds out[i] = a[i] - b[i]
        end
    end
else
    sub!(out, a, b) = broadcast!(-, out, a, b)
end
