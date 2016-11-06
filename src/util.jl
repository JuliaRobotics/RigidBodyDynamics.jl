# associative type that signifies an empty dictionary and does not allocate memory
immutable NullDict{K, V} <: Associative{K, V}
end
# Base.length(::NullDict) = 0
# Base.start(::NullDict) = 0
# Base.done(::NullDict, state) = true
# Base.get(::NullDict, key, default) = default
Base.haskey(::NullDict, k) = false

# ultimate sparse AbstractVector type that does not allocate memory
immutable NullVector{T} <: AbstractVector{T}
    length::Int64
end
Base.size(A::NullVector) = (A.length, )
Base.getindex{T}(A::NullVector{T}, i::Int) = zero(T)
# Base.setindex!(A::NullVector, v, i::Int) = error()
# Base.setindex!{N}(A::NullVector, v, I::Vararg{Int, N}) = error()
Base.linearindexing{T}(::Type{NullVector{T}}) = Base.LinearFast()

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

if VERSION < v"0.6-" # TODO: get rid of sub! after moving to 0.6. broadcast! allocates in 0.5; fixed in 0.6.
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


#=
Rotation utilities
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

function cross(a::SVector{3}, B::AbstractMatrix)
    hat(a) * B
end

rotate(x::SMatrix{3}, q::Quaternion) = rotation_matrix(q) * x

function rotate(x::SVector{3}, q::Quaternion)
    qret = q * Quaternion(zero(eltype(x)), x[1], x[2], x[3]) * inv(q)
    SVector(qret.v1, qret.v2, qret.v3)
end

function angle_axis_proper{T}(q::Quaternion{T})
    Θ = 2 * atan2(√(q.v1^2 + q.v2^2 + q.v3^2), q.s)
    axis = isapprox(Θ, zero(T)) ? SVector{3, T}(1, 0, 0) : SVector{3, T}(q.v1, q.v2, q.v3) / sin(Θ / 2)
    Θ, axis
end

function rotation_matrix{T}(q::Quaternion{T})
    sx, sy, sz = 2q.s*q.v1, 2q.s*q.v2, 2q.s*q.v3
    xx, xy, xz = 2q.v1^2, 2q.v1*q.v2, 2q.v1*q.v3
    yy, yz, zz = 2q.v2^2, 2q.v2*q.v3, 2q.v3^2
    @SMatrix [one(T)-(yy+zz) xy-sz xz+sy;
              xy+sz one(T)-(xx+zz) yz-sx;
              xz-sy yz+sx one(T)-(xx+yy)]
end

function rpy_to_quaternion(rpy::Vector)
    length(rpy) != 3 && error("wrong size")
    rpy2 = rpy / 2
    s = sin.(rpy2)
    c = cos.(rpy2)
    @inbounds qs = c[1]*c[2]*c[3] + s[1]*s[2]*s[3]
    @inbounds qx = s[1]*c[2]*c[3] - c[1]*s[2]*s[3]
    @inbounds qy = c[1]*s[2]*c[3] + s[1]*c[2]*s[3]
    @inbounds qz = c[1]*c[2]*s[3] - s[1]*s[2]*c[3]
    Quaternion(qs, qx, qy, qz)
end

function rotation_vector(q::Quaternion)
    Θ, axis = angle_axis_proper(q)
    ϕ = Θ * axis
end

function angle_axis_to_rotation_matrix{T}(angle::T, axis::AbstractVector{T})
    # axis is assumed to be normalized.
    @boundscheck length(axis) == 3 || error("axis has wrong size")
    # Rodrigues' formula:
    θ = angle
    S = hat(axis)
    R = eye(SMatrix{3, 3, T}) + sin(θ) * S + (1 - cos(θ)) * S^2
end

function rotation_vector_to_rotation_matrix{T}(ϕ::AbstractVector{T})
    θ = norm(ϕ)
    R = θ < eps(T) ? eye(SMatrix{3, 3, T}) : angle_axis_to_rotation_matrix(θ, ϕ * (1 / θ))
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
function rotation_vector_rate(rotation_vector::AbstractVector, angular_velocity_in_body::AbstractVector)
    ϕ = rotation_vector
    ω = angular_velocity_in_body
    @boundscheck length(ϕ) == 3 || error("ϕ has wrong length")
    @boundscheck length(ω) == 3 || error("ω has wrong length")
    Θ = norm(ϕ)
    ϕ̇ = ω
    if Θ > eps(Θ)
        ϕ̇ += (ϕ × ω) / 2 + 1 / Θ^2 * (1 - (Θ * sin(Θ)) / (2 * (1 - cos(Θ)))) * ϕ × (ϕ × ω)
    end
    ϕ̇
end
