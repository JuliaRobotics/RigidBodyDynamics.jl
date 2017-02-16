# Some operators involving a view of an SMatrix.
# TODO: make more efficient and less specific, or remove once StaticArrays does this.

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

@inline function colwise{S1, S2, T, L}(f, A::ContiguousSMatrixColumnView{S1, S2, T, L}, x::StaticVector)
    typeof(A)(colwise(f, parent(A), x), A.indexes, A.offset1, A.stride1)
end

@inline function colwise{S1, S2, T, L}(f, x::StaticVector, A::ContiguousSMatrixColumnView{S1, S2, T, L})
    typeof(A)(colwise(f, x, parent(A)), A.indexes, A.offset1, A.stride1)
end
