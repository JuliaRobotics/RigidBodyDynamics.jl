function parse_scalar{T}(::Type{T}, e::XMLElement, name::ASCIIString, default::ASCIIString)
    T(parse(e == nothing ? default : attribute(e, name)))
end

function parse_vector{T}(::Type{T}, e::XMLElement, name::ASCIIString, default::ASCIIString)
    usedefault = e == nothing || attribute(e, name) == nothing
    [T(parse(str)) for str in split(usedefault ? default : attribute(e, name), " ")]
end

function parse_inertia{T}(::Type{T}, e::XMLElement)
    ixx = parse_scalar(T, e, "ixx", "0")
    ixy = parse_scalar(T, e, "ixy", "0")
    ixz = parse_scalar(T, e, "ixz", "0")
    iyy = parse_scalar(T, e, "iyy", "0")
    iyz = parse_scalar(T, e, "iyz", "0")
    izz = parse_scalar(T, e, "izz", "0")
    return @fsa([ixx ixy ixz; ixy iyy iyz; ixz iyz izz])
end

function parse_pose{T}(::Type{T}, e::XMLElement)
    trans = Vec(parse_vector(T, e, "xyz", "0 0 0"))
    rpy = parse_vector(T, e, "rpy", "0 0 0")
    rot = rpy_to_quaternion(rpy)
    rot, trans
end

function parse_joint{T}(::Type{T}, e::XMLElement)
    name = attribute(e, "name")
    jointType = attribute(e, "type")
    if jointType == "revolute" || jointType == "continuous" # TODO: handle joint limits
        axis = Vec(parse_vector(T, find_element(e, "axis"), "xyz", "1 0 0"))
        return Joint(name, Revolute(axis))
    elseif jointType == "prismatic"
        axis = Vec(parse_vector(T, find_element(e, "axis"), "xyz", "1 0 0"))
        return Joint(name, Prismatic(axis))
    elseif jointType == "Floating"
        return Joint(name, QuaternionFloating())
    else
        error("joint type not recognized")
    end
end

function parse_inertia{T}(::Type{T}, e::XMLElement, frame::CartesianFrame3D)
    urdfFrame = CartesianFrame3D("inertia urdf helper")
    moment = parse_inertia(T, find_element(e, "inertia"))
    com = zero(Vec{3, T})
    mass = parse_scalar(T, find_element(e, "mass"), "value", "0")
    inertia = SpatialInertia(urdfFrame, moment, com, mass)
    pose = parse_pose(T, find_element(e, "origin"))
    return transform(inertia, Transform3D(urdfFrame, frame, pose...))
end

function parse_vertex{T}(mechanism::Mechanism{T}, vertex::TreeVertex{XMLElement, XMLElement})
    isroot(vertex) && error("unexpected non-root body")
    xmlJoint, xmlLink = vertex.edgeToParentData, vertex.vertexData

    joint = parse_joint(T, xmlJoint)

    parentName = attribute(find_element(xmlJoint, "parent"), "link")
    parent = findfirst(v -> RigidBodyDynamics.name(v.vertexData) == parentName, tree(mechanism)).vertexData
    pose = parse_pose(T, find_element(xmlJoint, "origin"))
    jointToParent = Transform3D(joint.frameBefore, default_frame(mechanism, parent), pose...)

    xmlInertial = find_element(xmlLink, "inertial")
    inertia = parse_inertia(T, xmlInertial, joint.frameAfter)
    linkname = attribute(xmlLink, "name")
    child = RigidBody(linkname, inertia)

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
    if length(roots) != 1
        error("Can only handle a single root")
    end
    tree = roots[1]

    # create mechanism from tree structure of XML elements
    mechanism = Mechanism{T}(attribute(tree.vertexData, "name"))
    for vertex in toposort(tree)[2 : end]
        parse_vertex(mechanism, vertex)
    end
    mechanism
end
