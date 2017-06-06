immutable ConstVector{T} <: AbstractVector{T}
    val::T
    length::Int64
end
Base.size(A::ConstVector) = (A.length, )
Base.getindex{T}(A::ConstVector{T}, i::Int) = (@boundscheck checkbounds(A, i); A.val)
@compat Base.IndexStyle{T}(::Type{ConstVector{T}}) = IndexLinear()


# type of a view of a vector
# TODO: a bit too specific
@compat const VectorSegment{T} = SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true}


"""
Views in Julia still allocate some memory (since they need to keep
a reference to the original array). This type allocates no memory
and does no bounds checking. Use it with caution.

Originally from https://github.com/mlubin/ReverseDiffSparse.jl/commit/8e3ade867581aad6ade7c898ada2ed58e0ad42bb.
"""
immutable UnsafeVectorView{T} <: AbstractVector{T}
    offset::Int
    len::Int
    ptr::Ptr{T}
end

@inline UnsafeVectorView(parent::Union{Vector, Base.FastContiguousSubArray}, range::UnitRange) = UnsafeVectorView(start(range) - 1, length(range), pointer(parent))
@inline Base.size(v::UnsafeVectorView) = (v.len,)
@inline Base.getindex(v::UnsafeVectorView, idx) = unsafe_load(v.ptr, idx + v.offset)
@inline Base.setindex!(v::UnsafeVectorView, value, idx) = unsafe_store!(v.ptr, value, idx + v.offset)
@inline Base.length(v::UnsafeVectorView) = v.len
@compat Base.IndexStyle{V <: UnsafeVectorView}(::Type{V}) = IndexLinear()

"""
UnsafeVectorView only works for isbits types. For other types, we're already
allocating lots of memory elsewhere, so creating a new SubArray is fine.
This function looks type-unstable, but the isbits(T) test can be evaluated
by the compiler, so the result is actually type-stable.

From https://github.com/rdeits/NNLS.jl/blob/0a9bf56774595b5735bc738723bd3cb94138c5bd/src/NNLS.jl#L218.
"""
@inline function fastview{T}(parent::Union{Vector{T}, Base.FastContiguousSubArray{T}}, range::UnitRange)
    if isbits(T)
        UnsafeVectorView(parent, range)
    else
        view(parent, range)
    end
end

@inline smatrix3x6view(mat::StaticMatrix) = _smatrix3x6view(Size(mat), mat)

@generated function _smatrix3x6view{S}(::Size{S}, mat::StaticMatrix)
    Snew = (S[1], 6)
    S[1] == 3 || error()
    (0 <= S[2] <= Snew[2]) || error()
    fillercols = Snew[2] - S[2]
    fillerlength = S[1] * fillercols
    T = eltype(mat)
    exprs = vcat([:(mat[$i]) for i = 1 : prod(S)], [:(zero($T)) for i = 1 : fillerlength])
    colrange =
    return quote
        Base.@_inline_meta
        @inbounds return view(similar_type(mat, Size($Snew))(tuple($(exprs...))), :, 1 : $S[2])
    end
end

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

macro rtti_dispatch(typeTuple, signature)
    @assert signature.head == :call
    @assert length(signature.args) > 1
    @assert typeTuple.head == :tuple

    f = signature.args[1]
    args = signature.args[2 : end]
    dispatchArg = args[1]
    otherArgs = args[2 : end]
    types = typeTuple.args

    ret = :(error("type not recognized"))
    for T in reverse(types)
        ret = Expr(:if, :(isa($dispatchArg, $T)), :(return $(f)($(dispatchArg)::$T, $(otherArgs...))), ret)
    end
    :($(esc(ret)))
end

@static if VERSION >= v"0.6-"
    @compat const ContiguousSMatrixColumnView{S1, S2, T, L} = SubArray{T,2,SMatrix{S1, S2, T, L},Tuple{Base.Slice{Base.OneTo{Int}},UnitRange{Int}},true}
else
    @compat const ContiguousSMatrixColumnView{S1, S2, T, L} = SubArray{T,2,SMatrix{S1, S2, T, L},Tuple{Colon,UnitRange{Int64}},true}
end


@compat const RotMatrix3{T} = RotMatrix{3, T, 9}

# TODO: use fusing broadcast instead of these functions in 0.6, where they don't allocate.
function sub!(out, a, b)
    @boundscheck length(out) == length(a) || error("size mismatch")
    @boundscheck length(out) == length(b) || error("size mismatch")
    @simd for i in eachindex(out)
        @inbounds out[i] = a[i] - b[i]
    end
end
@inline function scaleadd!(a::AbstractVector, b::AbstractVector, c::Number)
    @boundscheck length(a) == length(b) || error("size mismatch")
    @simd for i in eachindex(a)
        @inbounds a[i] += b[i] * c
    end
end


#=
Geometry utilities
=#
@inline function vector_to_skew_symmetric{T}(v::SVector{3, T})
    @SMatrix [zero(T) -v[3] v[2];
              v[3] zero(T) -v[1];
              -v[2] v[1] zero(T)]
end

const hat = vector_to_skew_symmetric

@inline function vector_to_skew_symmetric_squared(a::SVector{3})
    aSq1 = a[1] * a[1]
    aSq2 = a[2] * a[2]
    aSq3 = a[3] * a[3]
    b11 = -aSq2 - aSq3
    b12 = a[1] * a[2]
    b13 = a[1] * a[3]
    b22 = -aSq1 - aSq3
    b23 = a[2] * a[3]
    b33 = -aSq1 - aSq2
    @SMatrix [b11 b12 b13;
              b12 b22 b23;
              b13 b23 b33]
end

const hat_squared = vector_to_skew_symmetric_squared

# TODO: replace with future mapslices specialization, see https://github.com/JuliaArrays/StaticArrays.jl/pull/99
"""
    colwise(f, vec, mat)
Return a matrix `A` such that `A[:, i] == f(vec, mat[:, i])`.
"""
@generated function colwise(f, vec::StaticVector, mat::StaticArray)
    length(vec) == size(mat, 1) || throw(DimensionMismatch())
    exprs = [:(f(vec, mat[:, $j])) for j = 1:size(mat, 2)]
    return quote
        $(Expr(:meta, :inline))
        @inbounds return $(Expr(:call, hcat, exprs...))
    end
end

"""
    colwise(f, vec, mat)
Return a matrix `A` such that `A[:, i] == f(vec, mat[:, i])`.
"""
function colwise(f, vec::AbstractVector, mat::AbstractMatrix)
    mapslices(x -> f(vec, x), mat, (1,))
end

"""
    colwise(f, mat, vec)
Return a matrix `A` such that `A[:, i] == f(mat[:, i], vec)`.
"""
@generated function colwise(f, mat::StaticArray, vec::StaticVector)
    length(vec) == size(mat, 1) || throw(DimensionMismatch())
    exprs = [:(f(mat[:, $j], vec)) for j = 1:size(mat, 2)]
    return quote
        $(Expr(:meta, :inline))
        @inbounds return $(Expr(:call, hcat, exprs...))
    end
end

# The 'Bortz equation'.
# Bortz, John E. "A new mathematical formulation for strapdown inertial navigation."
# IEEE transactions on aerospace and electronic systems 1 (1971): 61-66.
#
# Or, interpreted in a Lie setting:
# d/dt(exp(ϕ(t))) = ̂(dexp(ϕ(t)) * ϕ̇(t)) * exp(ϕ(t)) (where dexp is the 'right trivialized' tangent of the exponential map)
# ̂(dexp(ϕ(t)) * ϕ̇(t)) = d/dt(exp(ϕ(t))) * exp(ϕ(t))⁻¹  (hat form of angular velocity in world frame)
#                      = ̂(exp(ϕ(t)) ω)      (with ω angular velocity in body frame)
# ϕ̇(t) = dexp⁻¹(ϕ(t)) * exp(ϕ(t)) * ω
function rotation_vector_rate{T}(rotation_vector::AbstractVector{T}, angular_velocity_in_body::AbstractVector{T})
    ϕ = rotation_vector
    ω = angular_velocity_in_body
    @boundscheck length(ϕ) == 3 || error("ϕ has wrong length")
    @boundscheck length(ω) == 3 || error("ω has wrong length")
    Θ = norm(ϕ)
    ϕ̇ = ω
    if Θ > eps(Θ)
        ϕ̇ += T(0.5) * (ϕ × ω) + 1 / Θ^2 * (1 - (Θ * sin(Θ)) / (2 * (1 - cos(Θ)))) * ϕ × (ϕ × ω)
    end
    ϕ̇
end

function angle_difference(a, b)
    mod2pi(b - a + pi) - π
end

function transform_spatial_motion{R <: Rotation{3}}(angular::SVector{3}, linear::SVector{3}, rot::R, trans::SVector{3})
    angular = rot * angular
    linear = rot * linear + cross(trans, angular)
    angular, linear
end

function mul_inertia(J, c, m, ω, v)
    angular = J * ω + cross(c, v)
    linear = m * v - cross(c, ω)
    angular, linear
end

# also known as 'spatial motion cross product'
@inline function se3_commutator(xω, xv, yω, yv)
    angular = cross(xω, yω)
    linear = cross(xω, yv) + cross(xv, yω)
    angular, linear
end

function quaternion_derivative(quat::Quat, angular_velocity_in_body::AbstractVector)
    @boundscheck length(angular_velocity_in_body) == 3 || error("size mismatch")
    q = quat
    ω = angular_velocity_in_body
    M = @SMatrix [
        -q.x -q.y -q.z;
         q.w -q.z  q.y;
         q.z  q.w -q.x;
        -q.y  q.x  q.w]
    M * (0.5 * ω)
end

function angular_velocity_in_body(quat::Quat, quat_derivative::AbstractVector)
    q = quat
    MInv = @SMatrix [
     -q.x  q.w  q.z -q.y;
     -q.y -q.z  q.w  q.x;
     -q.z  q.y -q.x  q.w]
    2 * (MInv * quat_derivative)
end

function findunique(f, A)
    results = find(f, A)
    length(results) == 0 && error("No results found.")
    length(results) > 1 && error("Multiple results found:\n$(A[results])")
    A[first(results)]
end
