## TypeSortedCollections addendum
# `foreach_with_extra_args` below is a hack to avoid allocations associated with creating closures over
# heap-allocated variables. Hopefully this will not be necessary in a future version of Julia.
for num_extra_args = 1 : 5
    extra_arg_syms = [Symbol("arg", i) for i = 1 : num_extra_args]
    @eval begin
        @generated function foreach_with_extra_args(f, $(extra_arg_syms...), A1::TypeSortedCollection{<:Any, N}, As::Union{<:TypeSortedCollection{<:Any, N}, AbstractVector}...) where {N}
            extra_args = $extra_arg_syms
            expr = Expr(:block)
            push!(expr.args, :(Base.@_inline_meta)) # required to achieve zero allocation
            push!(expr.args, :(leading_tsc = A1))
            push!(expr.args, :(@boundscheck TypeSortedCollections.lengths_match(A1, As...) || TypeSortedCollections.lengths_match_fail()))
            for i = 1 : N
                vali = Val(i)
                push!(expr.args, quote
                    let inds = leading_tsc.indices[$i]
                        @boundscheck TypeSortedCollections.indices_match($vali, inds, A1, As...) || TypeSortedCollections.indices_match_fail()
                        @inbounds for j in LinearIndices(inds)
                            vecindex = inds[j]
                            f($(extra_args...), TypeSortedCollections._getindex_all($vali, j, vecindex, A1, As...)...)
                        end
                    end
                end)
            end
            quote
                $expr
                nothing
            end
        end
    end
end
