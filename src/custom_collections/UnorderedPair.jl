struct UnorderedPair{T}
    a::T
    b::T
end

Base.hash(p::UnorderedPair, h::UInt) = hash(p.a, h) + hash(p.b, h)

function Base.:(==)(x::UnorderedPair, y::UnorderedPair)
    (x.a == y.a && x.b == y.b) || (x.a == y.b && x.b == y.a)
end
