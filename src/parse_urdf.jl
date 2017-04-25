function parse_scalar{T}(::Type{T}, e::XMLElement, name::String)
    T(parse(attribute(e, name)))
end

function parse_scalar{T}(::Type{T}, e::XMLElement, name::String, default::String)
    T(parse(e == nothing ? default : attribute(e, name)))
end

function parse_vector{T}(::Type{T}, e::Union{XMLElement, Void}, name::String, default::String)
    usedefault = e == nothing || attribute(e, name) == nothing # TODO: better handling of required attributes
    [T(parse(str)) for str in split(usedefault ? default : attribute(e, name))]
end

function parse_inertia{T}(::Type{T}, xmlInertia::XMLElement)
    ixx = parse_scalar(T, xmlInertia, "ixx", "0")
    ixy = parse_scalar(T, xmlInertia, "ixy", "0")
    ixz = parse_scalar(T, xmlInertia, "ixz", "0")
    iyy = parse_scalar(T, xmlInertia, "iyy", "0")
    iyz = parse_scalar(T, xmlInertia, "iyz", "0")
    izz = parse_scalar(T, xmlInertia, "izz", "0")
    @SMatrix [ixx ixy ixz; ixy iyy iyz; ixz iyz izz]
end

function parse_pose{T}(::Type{T}, xml_pose::Void)
    rot = eye(RotMatrix{3, T})
    trans = zero(SVector{3, T})
    rot, trans
end

function parse_pose{T}(::Type{T}, xml_pose::XMLElement)
    rpy = RotXYZ(parse_vector(T, xml_pose, "rpy", "0 0 0")...)
    rot = RotMatrix(rpy)
    trans = SVector{3}(parse_vector(T, xml_pose, "xyz", "0 0 0"))
    rot, trans
end

function parse_joint{T}(::Type{T}, xml_joint::XMLElement)
    name = attribute(xml_joint, "name")
    jointType = attribute(xml_joint, "type")
    if jointType == "revolute" || jointType == "continuous" # TODO: handle joint limits for revolute
        axis = SVector{3}(parse_vector(T, find_element(xml_joint, "axis"), "xyz", "1 0 0"))
        return Joint(name, Revolute(axis))
    elseif jointType == "prismatic"
        axis = SVector{3}(parse_vector(T, find_element(xml_joint, "axis"), "xyz", "1 0 0"))
        return Joint(name, Prismatic(axis))
    elseif jointType == "floating"
        return Joint(name, QuaternionFloating{T}())
    elseif jointType == "fixed"
        return Joint(name, Fixed{T}())
    else
        error("joint type $jointType not recognized")
    end
end

function parse_inertia{T}(::Type{T}, xml_inertial::XMLElement, frame::CartesianFrame3D)
    urdf_frame = CartesianFrame3D("inertia urdf helper")
    moment = parse_inertia(T, find_element(xml_inertial, "inertia"))
    com = zeros(SVector{3, T})
    mass = parse_scalar(T, find_element(xml_inertial, "mass"), "value", "0")
    inertia = SpatialInertia(urdf_frame, moment, com, mass)
    pose = parse_pose(T, find_element(xml_inertial, "origin"))
    transform(inertia, Transform3D(urdf_frame, frame, pose...))
end

function parse_body{T}(::Type{T}, xml_link::XMLElement, frame::CartesianFrame3D = CartesianFrame3D(attribute(xml_link, "name")))
    xml_inertial = find_element(xml_link, "inertial")
    inertia = xml_inertial == nothing ? zero(SpatialInertia{T}, frame) : parse_inertia(T, xml_inertial, frame)
    linkname = attribute(xml_link, "name") # TODO: make sure link name is unique
    RigidBody(linkname, inertia)
end

function parse_root_link{T}(mechanism::Mechanism{T}, xml_link::XMLElement)
    parent = root_body(mechanism)
    body = parse_body(T, xml_link)
    joint = Joint("$(name(body))_to_world", Fixed{T}())
    joint_to_parent = eye(STransform3D{T}, frame_before(joint), default_frame(parent))
    attach!(mechanism, parent, joint, joint_to_parent, body)
end

function parse_joint_and_link{T}(mechanism::Mechanism{T}, xml_parent::XMLElement, xml_child::XMLElement, xml_joint::XMLElement)
    parentname = attribute(xml_parent, "name")
    candidate_parents = collect(filter(b -> RigidBodyDynamics.name(b) == parentname, bodies(mechanism)))
    length(candidate_parents) == 1 || error("Duplicate name: $(parentname)")
    parent = first(candidate_parents)
    joint = parse_joint(T, xml_joint)
    pose = parse_pose(T, find_element(xml_joint, "origin"))
    joint_to_parent = Transform3D(frame_before(joint), default_frame(parent), pose...)
    body = parse_body(T, xml_child, frame_after(joint))
    attach!(mechanism, parent, joint, joint_to_parent, body)
end

"""
$(SIGNATURES)

Create a `Mechanism` by parsing a [URDF](http://wiki.ros.org/urdf) file.
"""
function parse_urdf{T}(scalartype::Type{T}, filename)
    xdoc = parse_file(filename)
    xroot = LightXML.root(xdoc)
    @assert LightXML.name(xroot) == "robot"

    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")

    # create graph structure of XML elements
    graph = DirectedGraph{Vertex{XMLElement}, Edge{XMLElement}}()
    vertices = Vertex.(xml_links)
    for vertex in vertices
        add_vertex!(graph, vertex)
    end
    name_to_vertex = Dict(attribute(data(v), "name") => v for v in vertices)
    for xml_joint in xml_joints
        parent = name_to_vertex[attribute(find_element(xml_joint, "parent"), "link")]
        child = name_to_vertex[attribute(find_element(xml_joint, "child"), "link")]
        add_edge!(graph, parent, child, Edge(xml_joint))
    end

    # create a spanning tree
    roots = collect(filter(v -> isempty(in_edges(v, graph)), vertices))
    length(roots) != 1 && error("Can only handle a single root")
    tree = SpanningTree(graph, first(roots))

    # create mechanism from spanning tree
    rootbody = RigidBody{T}("world")
    mechanism = Mechanism(rootbody)
    parse_root_link(mechanism, data(Graphs.root(tree)))
    for edge in edges(tree)
        parse_joint_and_link(mechanism, data(source(edge, tree)), data(target(edge, tree)), data(edge))
    end
    mechanism
end
