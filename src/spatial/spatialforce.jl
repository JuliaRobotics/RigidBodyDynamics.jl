for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval struct $ForceSpaceMatrix{A<:AbstractMatrix}
        frame::CartesianFrame3D
        angular::A
        linear::A

        @inline function $ForceSpaceMatrix{A}(frame::CartesianFrame3D, angular::AbstractMatrix, linear::AbstractMatrix) where {A<:AbstractMatrix}
            @boundscheck size(angular, 1) == 3 || throw(DimensionMismatch())
            @boundscheck size(linear, 1) == 3 || throw(DimensionMismatch())
            @boundscheck size(angular, 2) == size(linear, 2) || throw(DimensionMismatch())
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

for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval begin
        @inline function $ForceSpaceMatrix(frame::CartesianFrame3D, angular::A, linear::A) where {A<:AbstractMatrix}
            $ForceSpaceMatrix{A}(frame, angular, linear)
        end

        @inline function $ForceSpaceMatrix(frame::CartesianFrame3D, angular::A1, linear::A2) where {A1<:AbstractMatrix, A2<:AbstractMatrix}
            $ForceSpaceMatrix(frame, promote(angular, linear)...)
        end

        @inline function $ForceSpaceMatrix{A}(mat::$ForceSpaceMatrix) where A
            $ForceSpaceMatrix(mat.frame, A(angular(mat)), A(linear(mat)))
        end

        function Base.show(io::IO, m::$ForceSpaceMatrix)
            print(io, "$($(string(ForceSpaceMatrix))) expressed in \"$(string(m.frame))\":\n$(Array(m))")
        end

        @inline function transform(mat::$ForceSpaceMatrix, tf::Transform3D)
            @framecheck(mat.frame, tf.from)
            R = rotation(tf)
            Av = R * linear(mat)
            Aω = R * angular(mat) + colwise(×, translation(tf), Av)
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

    @inline function Momentum{T}(frame::CartesianFrame3D, angular::AbstractVector, linear::AbstractVector) where T
        new{T}(frame, angular, linear)
    end
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

    @inline function Wrench{T}(frame::CartesianFrame3D, angular::AbstractVector, linear::AbstractVector) where T
        new{T}(frame, angular, linear)
    end
end

for ForceSpaceElement in (:Momentum, :Wrench)
    @eval begin
        # Construct with possibly eltype-heterogeneous inputs
        @inline function $ForceSpaceElement(frame::CartesianFrame3D, angular::AbstractVector{T1}, linear::AbstractVector{T2}) where {T1, T2}
            $ForceSpaceElement{promote_type(T1, T2)}(frame, angular, linear)
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

        function Base.show(io::IO, f::$ForceSpaceElement)
            print(io, "$($(string(ForceSpaceElement))) expressed in \"$(string(f.frame))\":\nangular: $(angular(f)), linear: $(linear(f))")
        end

        function Base.zero(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, zero(SVector{3, T}), zero(SVector{3, T}))
        end

        Base.zero(f::$ForceSpaceElement) = zero(typeof(f), f.frame)

        function Random.rand(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end

        """
        $(SIGNATURES)

        Transform the $($(string(ForceSpaceElement))) to a different frame.
        """
        @inline function transform(f::$ForceSpaceElement, tf::Transform3D)
            @framecheck(f.frame, tf.from)
            rot = rotation(tf)
            lin = rot * linear(f)
            ang = rot * angular(f) + translation(tf) × lin
            $ForceSpaceElement(tf.to, ang, lin)
        end

        @inline function Base.:+(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, angular(f1) + angular(f2), linear(f1) + linear(f2))
        end

        @inline function Base.:-(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, angular(f1) - angular(f2), linear(f1) - linear(f2))
        end

        @inline Base.:-(f::$ForceSpaceElement) = $ForceSpaceElement(f.frame, -angular(f), -linear(f))

        function Base.isapprox(x::$ForceSpaceElement, y::$ForceSpaceElement; atol = 1e-12)
            x.frame == y.frame && isapprox(angular(x), angular(y), atol = atol) && isapprox(linear(x), linear(y), atol = atol)
        end
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
