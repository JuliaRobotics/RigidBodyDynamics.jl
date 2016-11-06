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
