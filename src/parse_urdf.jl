function parse_scalar{T}(::Type{T}, e::XMLElement, name::ASCIIString)
    T(parse(attribute(e, name)))
end

function parse_scalar{T}(::Type{T}, e::XMLElement, name::ASCIIString, default::ASCIIString)
    T(parse(e == nothing ? default : attribute(e, name)))
end

function parse_vector{T}(::Type{T}, e::XMLElement, name::ASCIIString, default::ASCIIString)
    usedefault = e == nothing || attribute(e, name) == nothing # TODO: better handling of required attributes
    [T(parse(str)) for str in split(usedefault ? default : attribute(e, name), " ")]
end

function parse_inertia{T}(::Type{T}, xmlInertia::XMLElement)
    ixx = parse_scalar(T, xmlInertia, "ixx", "0")
    ixy = parse_scalar(T, xmlInertia, "ixy", "0")
    ixz = parse_scalar(T, xmlInertia, "ixz", "0")
    iyy = parse_scalar(T, xmlInertia, "iyy", "0")
    iyz = parse_scalar(T, xmlInertia, "iyz", "0")
    izz = parse_scalar(T, xmlInertia, "izz", "0")
    return @fsa([ixx ixy ixz; ixy iyy iyz; ixz iyz izz])
end

function parse_pose{T}(::Type{T}, xmlPose::XMLElement)
    trans = Vec(parse_vector(T, xmlPose, "xyz", "0 0 0"))
    rpy = parse_vector(T, xmlPose, "rpy", "0 0 0")
    rot = rpy_to_quaternion(rpy)
    rot, trans
end

function parse_joint{T}(::Type{T}, xmlJoint::XMLElement)
    name = attribute(xmlJoint, "name")
    jointType = attribute(xmlJoint, "type")
    if jointType == "revolute" || jointType == "continuous" # TODO: handle joint limits for revolute
        axis = Vec(parse_vector(T, find_element(xmlJoint, "axis"), "xyz", "1 0 0"))
        return Joint(name, Revolute(axis))
    elseif jointType == "prismatic"
        axis = Vec(parse_vector(T, find_element(xmlJoint, "axis"), "xyz", "1 0 0"))
        return Joint(name, Prismatic(axis))
    elseif jointType == "floating"
        return Joint(name, QuaternionFloating())
    elseif jointType == "fixed"
        return Joint(name, Fixed())
    else
        error("joint type $jointType not recognized")
    end
end

function parse_inertia{T}(::Type{T}, xmlInertial::XMLElement, frame::CartesianFrame3D)
    urdfFrame = CartesianFrame3D("inertia urdf helper")
    moment = parse_inertia(T, find_element(xmlInertial, "inertia"))
    com = zero(Vec{3, T})
    mass = parse_scalar(T, find_element(xmlInertial, "mass"), "value", "0")
    inertia = SpatialInertia(urdfFrame, moment, com, mass)
    pose = parse_pose(T, find_element(xmlInertial, "origin"))
    return transform(inertia, Transform3D(urdfFrame, frame, pose...))
end

function parse_body{T}(::Type{T}, xmlLink::XMLElement, frame::CartesianFrame3D)
    xmlInertial = find_element(xmlLink, "inertial")
    inertia = parse_inertia(T, xmlInertial, frame)
    linkname = attribute(xmlLink, "name") # TODO: make sure link name is unique
    return RigidBody(linkname, inertia)
end

function parse_vertex{T}(mechanism::Mechanism{T}, vertex::TreeVertex{XMLElement, XMLElement})
    isroot(vertex) && error("unexpected non-root body")
    xmlJoint, xmlLink = vertex.edgeToParentData, vertex.vertexData
    parentName = attribute(find_element(xmlJoint, "parent"), "link")
    parent = findfirst(v -> RigidBodyDynamics.name(v.vertexData) == parentName, tree(mechanism)).vertexData
    joint = parse_joint(T, xmlJoint)
    pose = parse_pose(T, find_element(xmlJoint, "origin"))
    jointToParent = Transform3D(joint.frameBefore, default_frame(mechanism, parent), pose...)
    child = parse_body(T, xmlLink, joint.frameAfter)
    attach!(mechanism, parent, joint, jointToParent, child)
end

function parse_urdf{T}(::Type{T}, filename)
    xdoc = parse_file(filename)
    xroot = root(xdoc)
    @assert LightXML.name(xroot) == "robot"

    xmlLinks = get_elements_by_tagname(xroot, "link")
    xmlJoints = get_elements_by_tagname(xroot, "joint")

    # create tree structure of XML elements
    vertices = [TreeVertex{XMLElement, XMLElement}(e) for e in xmlLinks]
    nameToVertex = [attribute(v.vertexData, "name")::ASCIIString => v::TreeVertex{XMLElement, XMLElement} for v in vertices]
    for xmlJoint in xmlJoints
        parent = nameToVertex[attribute(find_element(xmlJoint, "parent"), "link")]
        child = nameToVertex[attribute(find_element(xmlJoint, "child"), "link")]
        push!(parent.children, child)
        child.parent = parent
        child.edgeToParentData = xmlJoint
    end
    roots = filter(isroot, vertices)
    length(roots) != 1 && error("Can only handle a single root")
    tree = roots[1]

    # create mechanism from tree structure of XML elements
    mechanism = Mechanism{T}(attribute(tree.vertexData, "name"))
    for vertex in toposort(tree)[2 : end]
        parse_vertex(mechanism, vertex)
    end
    mechanism
end
