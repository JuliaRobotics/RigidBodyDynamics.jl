# Some operators involving a view of an SMatrix.
# TODO: make more efficient and less specific, or remove once StaticArrays does this.
import Base: *, +, -

function *{S1, S2, T, L}(A::StaticMatrix, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = A * parent(B)
    typeof(B)(data, (:, B.indexes[2]), B.offset1, B.stride1)
end

function +{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L}, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    @boundscheck size(A) == size(B) || error("size mismatch")
    data = parent(A) + parent(B)
    typeof(A)(data, A.indexes, A.offset1, A.stride1)
end

function -{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L}, B::ContiguousSMatrixColumnView{S1, S2, T, L})
    @boundscheck size(A) == size(B) || error("size mismatch")
    data = parent(A) - parent(B)
    typeof(A)(data, A.indexes, A.offset1, A.stride1)
end

function -{S1, S2, T, L}(A::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = -parent(A)
    typeof(A)(data, A.indexes, A.offset1, A.stride1)
end

function *{S1, S2, T, L}(s::Number, A::ContiguousSMatrixColumnView{S1, S2, T, L})
    data = s * parent(A)
    typeof(A)(data, A.indexes, A.offset1, A.stride1)
end

# FIXME: hack to get around ambiguities
_mul(a, b) = a * b

# TODO: too specific
function _mul{S1, S2, TA, L, Tb}(
        A::ContiguousSMatrixColumnView{S1, S2, TA, L},
        b::Union{StridedVector{Tb}, UnsafeVectorView{Tb}})
    @boundscheck size(A, 2) == size(b, 1) || error("size mismatch")
    ret = zeros(SVector{S1, promote_type(TA, Tb)})
    for i = 1 : size(A, 2)
        @inbounds bi = b[i]
        Acol = SVector{S1, TA}(view(A, :, i))
        ret = ret + Acol * bi
    end
    ret
end

# FIXME: just use convert when https://github.com/FugroRoames/Rotations.jl/pull/16 is in.
@inline function angle_axis_to_rotation_matrix(aa::AngleAxis)
    # Rodrigues' rotation formula.
    T = eltype(aa)

    s = sin(aa.theta)
    c = cos(aa.theta)
    c1 = one(T) - c

    c1x2 = c1 * aa.axis_x^2
    c1y2 = c1 * aa.axis_y^2
    c1z2 = c1 * aa.axis_z^2

    c1xy = c1 * aa.axis_x * aa.axis_y
    c1xz = c1 * aa.axis_x * aa.axis_z
    c1yz = c1 * aa.axis_y * aa.axis_z

    sx = s * aa.axis_x
    sy = s * aa.axis_y
    sz = s * aa.axis_z

    # Note that the RotMatrix constructor argument order makes this look transposed:
    RotMatrix(one(T) - c1y2 - c1z2, c1xy + sz, c1xz - sy,
      c1xy - sz, one(T) - c1x2 - c1z2, c1yz + sx,
      c1xz + sy, c1yz - sx, one(T) - c1x2 - c1y2)
end
