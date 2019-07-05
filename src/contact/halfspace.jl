struct HalfSpace{N, T}
    outward_normal::SVector{N, T}
    offset::T

    function HalfSpace(outward_normal::SVector{N, T}, offset::T) where {N, T}
        new{N, T}(normalize(outward_normal), offset)
    end
end

function HalfSpace(outward_normal::StaticVector{N}, offset) where {N}
    T = promote_type(typeof(offset), eltype(outward_normal))
    HalfSpace(SVector{N, T}(outward_normal), T(offset))
end

function HalfSpace(outward_normal::StaticVector{N}, point::StaticVector{N}) where N
    HalfSpace(outward_normal, point ⋅ outward_normal)
end

function (tf::AffineMap)(halfspace::HalfSpace)
    linear = LinearMap(transform_deriv(tf, 0))
    normal = linear(halfspace.outward_normal)
    offset = halfspace.offset + normal ⋅ tf.translation
    HalfSpace(normal, offset)
end

function (tf::Translation)(halfspace::HalfSpace)
    normal = halfspace.outward_normal
    offset = halfspace.offset + normal ⋅ tf.translation
    HalfSpace(normal, offset)
end
