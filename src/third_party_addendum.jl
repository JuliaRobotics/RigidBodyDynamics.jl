function rotate{N, T}(x::Mat{3, N, T}, q::Quaternion{T})
    return Mat(rotationmatrix(q)) * x
end

function rotate{T}(x::Vec{3, T}, q::Quaternion{T})
    qret = q * Quaternion(0, x[1], x[2], x[3]) * inv(q)
    return Vec(qret.v1, qret.v2, qret.v3)
end

function isapprox_tol{FSA <: FixedArray, A <: Union{Array, FixedArray}}(a::FSA, b::A; atol::Real = 0)
    for i=1:length(a)
        !isapprox(a[i], b[i]; atol = atol) && return false
    end
    true
end

angle_axis_proper(q::Quaternion) = angle_proper(q), axis_proper(q)

angle_proper(q::Quaternion) = 2 * acos(real(Quaternions.normalize(q)))

function axis_proper(q::Quaternion)
    q = Quaternions.normalize(q)
    s = sin(angle(q) / 2)
        abs(s) > 0 ?
        [q.v1, q.v2, q.v3] / s :
        [1.0, 0.0, 0.0]
end

function rotationmatrix_normalized_fsa{T}(q::Quaternion{T})
    sx, sy, sz = 2q.s*q.v1, 2q.s*q.v2, 2q.s*q.v3
    xx, xy, xz = 2q.v1^2, 2q.v1*q.v2, 2q.v1*q.v3
    yy, yz, zz = 2q.v2^2, 2q.v2*q.v3, 2q.v3^2
    return Mat{3, 3, T}((1-(yy+zz), xy+sz, xz-sy), (xy-sz, 1-(xx+zz), yz+sx), (xz+sy, yz-sx, 1-(xx+yy)))
end

function rpy_to_quaternion(rpy::Vector)
    rpy2 = rpy / 2
    s = sin(rpy2)
    c = cos(rpy2)
    return Quaternion(
        c[1]*c[2]*c[3] + s[1]*s[2]*s[3],
        s[1]*c[2]*c[3] - c[1]*s[2]*s[3],
        c[1]*s[2]*c[3] + s[1]*c[2]*s[3],
        c[1]*c[2]*s[3] - s[1]*s[2]*c[3])
end

hcat(head::Mat) = head
function hcat(head::Mat, tail::Mat...)
    tailhcat = hcat(tail...)
    if isempty(head) && isempty(tailhcat)
        return zero(head) # TODO: check size match
    else
        return Mat((head._..., tailhcat._...))
    end
end

function cross{N, T}(a::Vec{3, T}, B::Mat{3, N, T})
    Mat(map((col) -> cross(a, Vec(col))._::Tuple{T, T, T}, B._))::Mat{3, N, T}
end
