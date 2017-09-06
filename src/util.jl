# Copy block of matrix. TODO: use higher level abstraction once it's fast
@inline function set_matrix_block!(out::AbstractMatrix, irange::UnitRange, jrange::UnitRange, part::AbstractMatrix)
    for col in 1 : size(part, 2), row in 1 : size(part, 1)
        out[irange[row], jrange[col]] = part[row, col]
    end
end

# zero block of a matrix. TODO: use higher level abstraction once it's fast
@inline function zero_matrix_block!(out::AbstractMatrix, irange::UnitRange, jrange::UnitRange)
    for col in jrange, row in irange
        out[row, col] = 0
    end
end

# TODO: higher level abstraction once it's as fast
for T in (:GeometricJacobian, :MomentumMatrix)
    @eval @inline function set_cols!(out::$T, vrange::UnitRange, part::$T)
        @framecheck out.frame part.frame
        for col in 1 : size(part, 2)
            outcol = vrange[col]
            angular(out)[1, outcol] = angular(part)[1, col]
            angular(out)[2, outcol] = angular(part)[2, col]
            angular(out)[3, outcol] = angular(part)[3, col]
            linear(out)[1, outcol] = linear(part)[1, col]
            linear(out)[2, outcol] = linear(part)[2, col]
            linear(out)[3, outcol] = linear(part)[3, col]
        end
    end
end

@inline function zero_cols!(out::GeometricJacobian, vrange::UnitRange)
    for j in vrange # TODO: use higher level abstraction once it's as fast
        angular(out)[1, j] = angular(out)[2, j] = angular(out)[3, j] = 0;
        linear(out)[1, j] = linear(out)[2, j] = linear(out)[3, j] = 0;
    end
end

## findunique
function findunique(f, A)
    results = find(f, A)
    length(results) == 0 && error("No results found.")
    length(results) > 1 && error("Multiple results found:\n$(A[results])")
    A[first(results)]
end


# Cached download
const module_tempdir = joinpath(Base.tempdir(), string(module_name(@__MODULE__)))

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
