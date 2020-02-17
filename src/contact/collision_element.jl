struct CollisionElement{T, G}
    bodyid::BodyID
    transform::T # to body's default frame
    geometry::G
end

function CollisionElement(body::RigidBody, transform, geometry)
    CollisionElement(BodyID(body), transform, geometry)
end

function CollisionElement(body::RigidBody, frame::CartesianFrame3D, geometry)
    CollisionElement(body, frame_definition(body, frame), geometry)
end
