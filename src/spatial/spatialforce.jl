for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval struct $ForceSpaceMatrix{A<:AbstractMatrix}
        frame::CartesianFrame3D
        angular::A
        linear::A

        @inline function $ForceSpaceMatrix(frame::CartesianFrame3D, angular::A, linear::A) where {A<:AbstractMatrix}
            @boundscheck size(angular, 1) == 3 || error("size mismatch")
            @boundscheck size(linear, 1) == 3 || error("size mismatch")
            @boundscheck size(angular, 2) == size(linear, 2) || error("size mismatch")
            new{A}(frame, angular, linear)
        end
    end
end

"""
$(TYPEDEF)

A momentum matrix maps a joint velocity vector to momentum.

This is a slight generalization of the centroidal momentum matrix
(Orin, Goswami, "Centroidal momentum matrix of a humanoid robot: Structure and properties.")
in that the matrix (and hence the corresponding total momentum) need not be
expressed in a centroidal frame.
"""
MomentumMatrix

# WrenchSubspace is the return type of e.g. constraint_wrench_subspace(::Joint, ...)
const WrenchSubspace{T} = WrenchMatrix{ContiguousSMatrixColumnView{3, 6, T, 18}}
function WrenchSubspace(frame::CartesianFrame3D, angular, linear)
    WrenchMatrix(frame, smatrix3x6view(angular), smatrix3x6view(linear))
end
WrenchSubspace(mat::WrenchMatrix) = WrenchSubspace(mat.frame, mat.angular, mat.linear)

for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval begin
        Base.convert(::Type{$ForceSpaceMatrix{A}}, mat::$ForceSpaceMatrix{A}) where {A} = mat
        function Base.convert(::Type{$ForceSpaceMatrix{A}}, mat::$ForceSpaceMatrix) where {A}
            $ForceSpaceMatrix(mat.frame, convert(A, mat.angular), convert(A, mat.linear))
        end
        Base.Array(mat::$ForceSpaceMatrix) = [Array(mat.angular); Array(mat.linear)]

        Base.eltype(::Type{$ForceSpaceMatrix{A}}) where {A} = eltype(A)
        Base.size(mat::$ForceSpaceMatrix) = (6, size(mat.angular, 2))
        Base.size(mat::$ForceSpaceMatrix, d) = size(mat)[d]
        angular(mat::$ForceSpaceMatrix) = mat.angular
        linear(mat::$ForceSpaceMatrix) = mat.linear

        function Base.show(io::IO, m::$ForceSpaceMatrix)
            print(io, "$($(string(ForceSpaceMatrix))) expressed in \"$(name(m.frame))\":\n$(Array(m))")
        end

        function transform(mat::$ForceSpaceMatrix, tf::Transform3D)
            @framecheck(mat.frame, tf.from)
            R = rotation(tf)
            Av = R * linear(mat)
            Aω = R * angular(mat) + colwise(cross, translation(tf), Av)
            $ForceSpaceMatrix(tf.to, Aω, Av)
        end
    end
end

"""
$(TYPEDEF)

A `Momentum` is the product of a `SpatialInertia` and a `Twist`, i.e.
```math
h^i =
\\left(\\begin{array}{c}
k^{i}\\\\
l^{i}
\\end{array}\\right) =
I^i T^{i, j}_k
```
where ``I^i`` is the spatial inertia of a given body expressed in frame ``i``,
and ``T^{i, j}_k`` is the twist of frame ``k`` (attached to the body) with
respect to inertial frame ``j``, expressed in frame ``i``. ``k^i`` is the
angular momentum and ``l^i`` is the linear momentum.
"""
struct Momentum{T}
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end

"""
$(TYPEDEF)

A wrench represents a system of forces.

The wrench ``w^i`` expressed in frame ``i`` is defined as
```math
w^{i} =
\\left(\\begin{array}{c}
\\tau^{i}\\\\
f^{i}
\\end{array}\\right) =
\\sum_{j}\\left(\\begin{array}{c}
r_{j}^{i}\\times f_{j}^{i}\\\\
f_{j}^{i}
\\end{array}\\right)
```
where the ``f_{j}^{i}`` are forces expressed in frame ``i``, exerted at
positions ``r_{j}^{i}``. ``\\tau^i`` is the total torque and ``f^i`` is the
total force.
"""
struct Wrench{T}
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end

for ForceSpaceElement in (:Momentum, :Wrench)
    @eval begin
        function $ForceSpaceElement(frame::CartesianFrame3D, angular::AbstractVector{T1}, linear::AbstractVector{T2}) where {T1, T2}
            T = promote_type(T1, T2)
            $ForceSpaceElement{T}(frame, convert(SVector{3, T}, angular), convert(SVector{3, T}, linear))
        end

        """
        $(SIGNATURES)

        Create a $($(string(ForceSpaceElement))) given the angular and linear
        components, which should be expressed in the same frame.
        """
        function $ForceSpaceElement(angular::FreeVector3D, linear::FreeVector3D)
            @framecheck angular.frame linear.frame
            $ForceSpaceElement(angular.frame, angular.v, linear.v)
        end

        Base.convert(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement{T}) where {T} = f

        function Base.convert(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement) where {T}
            $ForceSpaceElement(f.frame, convert(SVector{3, T}, f.angular), convert(SVector{3, T}, f.linear))
        end

        Base.eltype(::Type{$ForceSpaceElement{T}}) where {T} = T
        StaticArrays.similar_type(::Type{$ForceSpaceElement{T1}}, ::Type{T2}) where {T1, T2} = $ForceSpaceElement{T2} # FIXME: lose this

        function Base.show(io::IO, f::$ForceSpaceElement)
            print(io, "$($(string(ForceSpaceElement))) expressed in \"$(name(f.frame))\":\nangular: $(f.angular), linear: $(f.linear)")
        end

        function Base.zero(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
        end

        Base.zero(f::$ForceSpaceElement) = zero(typeof(f), f.frame)

        function Random.rand(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end

        """
        $(SIGNATURES)

        Transform the $($(string(ForceSpaceElement))) to a different frame.
        """
        function transform(f::$ForceSpaceElement, tf::Transform3D)
            @framecheck(f.frame, tf.from)
            rot = rotation(tf)
            linear = rot * f.linear
            angular = rot * f.angular + cross(translation(tf), linear)
            $ForceSpaceElement(tf.to, angular, linear)
        end

        function Base.:+(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular + f2.angular, f1.linear + f2.linear)
        end

        function Base.:-(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular - f2.angular, f1.linear - f2.linear)
        end

        Base.:-(f::$ForceSpaceElement) = $ForceSpaceElement(f.frame, -f.angular, -f.linear)

        Base.Array(f::$ForceSpaceElement) = [f.angular...; f.linear...]
        Base.isapprox(x::$ForceSpaceElement, y::$ForceSpaceElement; atol = 1e-12) = x.frame == y.frame && isapprox(x.angular, y.angular, atol = atol) && isapprox(x.linear, y.linear, atol = atol)
    end
end

# Wrench-specific functions
"""
$(SIGNATURES)

Create a Wrench from a force, ``f``, and the application point of the force, ``r``.
The torque part of the wrench will be computed as ``r \\times f``. The force
and the application point should be expressed in the same frame.
"""
Wrench(application_point::Point3D, force::FreeVector3D) = Wrench(application_point × force, force)
