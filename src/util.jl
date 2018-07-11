# TODO: higher level abstraction once it's as fast
for T in (:GeometricJacobian, :MomentumMatrix)
    @eval @inline function set_col!(dest::$T, col::Integer, src::$T)
        @framecheck dest.frame src.frame
        @boundscheck size(src, 2) == 1 || throw(ArgumentError())
        @boundscheck col ∈ Base.OneTo(size(dest, 2)) || throw(DimensionMismatch())
        @inbounds begin
            start = LinearIndices(dest.angular)[1, col]
            dest.angular[start] = src.angular[1]
            dest.angular[start + 1] = src.angular[2]
            dest.angular[start + 2] = src.angular[3]
            dest.linear[start] = src.linear[1]
            dest.linear[start + 1] = src.linear[2]
            dest.linear[start + 2] = src.linear[3]
        end
    end
end


## findunique
function findunique(f, A::AbstractArray)
    i = findfirst(f, A)
    i === nothing && error("No results found.")
    findnext(f, A, i + 1) === nothing || error("Multiple results found.")
    @inbounds return A[i]
end


# Cached download
const module_tempdir = joinpath(Base.tempdir(), string(nameof(@__MODULE__)))

function cached_download(url::String, local_file_name::String, cache_dir::String = joinpath(module_tempdir, string(hash(url))))
    if !ispath(cache_dir)
        mkpath(cache_dir)
    end
    full_cache_path = joinpath(cache_dir, local_file_name)
    if !isfile(full_cache_path)
        download(url, full_cache_path)
    end
    full_cache_path
end


## VectorSegment: type of a view of a vector
const VectorSegment{T} = SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true} # TODO: a bit too specific

quatnorm(quat::Quat) = sqrt(quat.w^2 + quat.x^2 + quat.y^2 + quat.z^2)


## Modification count stuff
function modcount end

struct ModificationCountMismatch <: Exception
    msg::String
end
Base.showerror(io::IO, ex::ModificationCountMismatch) = print(io, "ModificationCountMismatch: $(ex.msg)")

macro modcountcheck(a, b)
    quote
        modcount($(esc(a))) == modcount($(esc(b))) || modcount_check_fail($(QuoteNode(a)), $(QuoteNode(b)), $(esc(a)), $(esc(b)))
    end
end

@noinline function modcount_check_fail(asym, bsym, a, b)
    amod = modcount(a)
    bmod = modcount(b)
    msg = "Modification count of '$(string(asym))' ($amod) does not match modification count of '$(string(bsym))' ($bmod)."
    throw(ModificationCountMismatch(msg))
end


# Bounds
"""
$(TYPEDEF)

Bounds is a scalar-like type representing a closed interval from ``lower`` to
``upper``. To indicate that a vector of values falls with some range, use a
``Vector{Bounds{T}}``.
"""
struct Bounds{T}
    lower::T
    upper::T

    function Bounds{T}(lower, upper) where T
        @assert lower <= upper
        new{T}(lower, upper)
    end
    Bounds{T}() where {T} = new{T}(typemin(T), typemax(T))
end

Bounds(lower::T1, upper::T2) where {T1, T2} = Bounds{promote_type(T1, T2)}(lower, upper)

upper(b::Bounds) = b.upper
lower(b::Bounds) = b.lower
Base.:(==)(b1::Bounds, b2::Bounds) = b1.lower == b2.lower && b1.upper == b2.upper
Base.:-(b::Bounds) = Bounds(-b.upper, -b.lower)
Base.show(io::IO, b::Bounds) = print(io, "(", lower(b), ", ", upper(b), ")")
Base.convert(::Type{Bounds{T1}}, b::Bounds{T2}) where {T1, T2} = Bounds{T1}(convert(T1, lower(b)), convert(T1, upper(b)))
Base.broadcastable(b::Bounds) = Ref(b)

"""
$(SIGNATURES)

Return the closest value to ``x`` within the interval described by ``b``.
"""
Base.clamp(x, b::Bounds) = clamp(x, b.lower, b.upper)
Base.intersect(b1::Bounds, b2::Bounds) = Bounds(max(b1.lower, b2.lower), min(b1.upper, b2.upper))

# Create a new Int-backed index type, along with necessary methods to support creating ranges
macro indextype(ID)
    esc(quote
        struct $ID <: Integer
            value::Int
        end
        $ID(id::$ID) = id
        Base.hash(i::$ID, h::UInt) = hash(i.value, h)
        Base.convert(::Type{Int}, i::$ID) = i.value
        Base.Integer(i::$ID) = i.value
        Base.Int(i::$ID) = i.value
        Base.convert(::Type{$ID}, i::Integer) = $ID(i)
        Base.promote_type(::Type{Int}, ::Type{$ID}) = $ID
        Base.promote_type(::Type{$ID}, ::Type{Int}) = $ID
        Base.:<(x::$ID, y::$ID) = x.value < y.value
        Base.:<=(x::$ID, y::$ID) = x.value <= y.value
        Base.:-(x::$ID, y::$ID) = x.value - y.value
        Base.:+(x::$ID, y::Int) = $ID(x.value + y)
    end)
end
