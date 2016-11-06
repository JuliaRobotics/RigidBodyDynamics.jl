function vector_to_skew_symmetric{T}(v::SVector{3, T})
    @SMatrix [zero(T) -v[3] v[2];
              v[3] zero(T) -v[1];
              -v[2] v[1] zero(T)]
end

const hat = vector_to_skew_symmetric

function cross(a::SVector{3}, B::AbstractMatrix)
    vector_to_skew_symmetric(a) * B
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

# Some operators involving a view of an SMatrix.
# TODO: make more efficient and less specific, or remove once StaticArrays does this.
import Base: *, +, -

function *{S1, S2, T, L}(A::StaticMatrix, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = A * parent(B)
    view(data, B.indexes[1], B.indexes[2])
end

function +{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L}, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    @boundscheck size(A) == size(B)
    data = parent(A) + parent(B)
    view(data, A.indexes[1], A.indexes[2])
end

function -{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L}, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    @boundscheck size(A) == size(B)
    data = parent(A) - parent(B)
    view(data, A.indexes[1], A.indexes[2])
end

function -{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = -parent(A)
    view(data, A.indexes[1], A.indexes[2])
end

function *{S1, S2, T, L}(s::Number, A::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = s * parent(A)
    view(data, A.indexes[1], A.indexes[2])
end

# FIXME: hack to get around ambiguities
_mul(a, b) = a * b

# TODO: too specific
function _mul{S1, S2, TA, L, Tb}(
        A::ContiguousSMatrixColumnView{S1, S2, TA, L},
        b::StridedVector{Tb})
    @boundscheck @assert size(A, 2) == size(b, 1)
    ret = zeros(SVector{S1, promote_type(TA, Tb)})
    for i = 1 : size(A, 2)
        @inbounds bi = b[i]
        Acol = SVector{S1, TA}(view(A, :, i))
        ret = ret + Acol * bi
    end
    ret
end
