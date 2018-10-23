for SpatialVector in (:Twist, :SpatialAcceleration, :Momentum, :Wrench)
    @eval begin
        # Basics
        Base.eltype(::Type{$SpatialVector{T}}) where {T} = T
        angular(v::$SpatialVector) = v.angular
        linear(v::$SpatialVector) = v.linear

        # Construct/convert given another spatial vector
        $SpatialVector{T}(v::$SpatialVector{T}) where {T} = v
        Base.convert(::Type{V}, v::$SpatialVector) where {V<:$SpatialVector} = V(v)

        # Construct/convert to SVector
        StaticArrays.SVector{6, T}(v::$SpatialVector) where {T} = convert(SVector{6, T}, [angular(v); linear(v)])
        StaticArrays.SVector{6}(v::$SpatialVector{X}) where {X} = SVector{6, X}(v)
        StaticArrays.SVector(v::$SpatialVector) = SVector{6}(v)
        StaticArrays.SArray(v::$SpatialVector) = SVector(v)
        Base.convert(::Type{SA}, v::$SpatialVector) where {SA <: SArray} = SA(v)

        function Base.Array(v::$SpatialVector)
            Base.depwarn("Array(v) has been deprecated. Please use SVector(v) instead.", :Array)
            SVector(v)
        end
    end
end

for SpatialMatrix in (:GeometricJacobian, :MomentumMatrix, :WrenchMatrix)
    @eval begin
        # Basics
        Base.eltype(::Type{$SpatialMatrix{A}}) where {T, A<:AbstractMatrix{T}} = T
        Base.size(m::$SpatialMatrix) = (6, size(angular(m), 2))
        Base.size(m::$SpatialMatrix, d) = size(m)[d]
        Base.transpose(m::$SpatialMatrix) = Transpose(m)
        angular(m::$SpatialMatrix) = m.angular
        linear(m::$SpatialMatrix) = m.linear

        # Construct/convert given another spatial matrix
        $SpatialMatrix(m::$SpatialMatrix{A}) where {A} = SpatialMatrix{A}(m)
        Base.convert(::Type{$SpatialMatrix{A}}, m::$SpatialMatrix) where {A} = $SpatialMatrix{A}(m)
        Base.convert(::Type{$SpatialMatrix{A}}, m::$SpatialMatrix{A}) where {A} = m

        # Construct/convert to Matrix
        (::Type{A})(m::$SpatialMatrix) where {A<:Array} = convert(A, [angular(m); linear(m)])
        Base.convert(::Type{A}, m::$SpatialMatrix) where {A<:Array} = A(m)
    end
end
