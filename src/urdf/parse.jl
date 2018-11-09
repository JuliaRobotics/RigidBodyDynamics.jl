function parse_scalar(::Type{T}, e::XMLElement, name::String) where {T}
    parse(T, attribute(e, name))
end

function parse_scalar(::Type{T}, e::XMLElement, name::String, default::String) where {T}
    parse(T, e == nothing ? default : attribute(e, name))
end

function parse_vector(::Type{T}, e::Union{XMLElement, Nothing}, name::String, default::String) where {T}
    usedefault = e == nothing || attribute(e, name) == nothing # TODO: better handling of required attributes
    [parse(T, str) for str in split(usedefault ? default : attribute(e, name))]
end

function parse_inertia(::Type{T}, xml_inertia::XMLElement) where {T}
    ixx = parse_scalar(T, xml_inertia, "ixx", "0")
    ixy = parse_scalar(T, xml_inertia, "ixy", "0")
    ixz = parse_scalar(T, xml_inertia, "ixz", "0")
    iyy = parse_scalar(T, xml_inertia, "iyy", "0")
    iyz = parse_scalar(T, xml_inertia, "iyz", "0")
    izz = parse_scalar(T, xml_inertia, "izz", "0")
    @SMatrix [ixx ixy ixz; ixy iyy iyz; ixz iyz izz]
end

function parse_pose(::Type{T}, xml_pose::Nothing) where {T}
    rot = one(RotMatrix3{T})
    trans = zero(SVector{3, T})
    rot, trans
end

function parse_pose(::Type{T}, xml_pose::XMLElement) where {T}
    rpy = parse_vector(T, xml_pose, "rpy", "0 0 0")
    rot = RotMatrix(RotZYX(rpy[3], rpy[2], rpy[1]))
    trans = SVector{3}(parse_vector(T, xml_pose, "xyz", "0 0 0"))
    rot, trans
end

function parse_joint_type(::Type{T}, xml_joint::XMLElement,
        floating_joint_type::Type{<:JointType{T}}, revolute_joint_type::Type{<:JointType{T}}) where {T}
    joint_type = attribute(xml_joint, "type")
    if joint_type == "revolute" || joint_type == "continuous" # TODO: handle joint limits for revolute
        axis = SVector{3}(parse_vector(T, find_element(xml_joint, "axis"), "xyz", "1 0 0"))
        return revolute_joint_type(axis)
    elseif joint_type == "prismatic"
        axis = SVector{3}(parse_vector(T, find_element(xml_joint, "axis"), "xyz", "1 0 0"))
        return Prismatic(axis)
    elseif joint_type == "floating"
        return floating_joint_type()
    elseif joint_type == "fixed"
        return Fixed{T}()
    elseif joint_type == "planar"
        urdf_axis = SVector{3}(parse_vector(T, find_element(xml_joint, "axis"), "xyz", "1 0 0"))
        # The URDF spec says that a planar joint allows motion in a
        # plane perpendicular to the axis.
        R = Rotations.rotation_between(SVector(0, 0, 1), urdf_axis)
        return Planar{T}(R * SVector(1, 0, 0), R * SVector(0, 1, 0))
    else
        error("joint type $(string(joint_type)) not recognized")
    end
end

function parse_joint_bounds(jtype::JT, xml_joint::XMLElement) where {T, JT <: JointType{T}}
    position_bounds = fill(Bounds{T}(), num_positions(jtype))
    velocity_bounds = fill(Bounds{T}(), num_velocities(jtype))
    effort_bounds = fill(Bounds{T}(), num_velocities(jtype))
    for element in get_elements_by_tagname(xml_joint, "limit")
        if has_attribute(element, "lower")
            position_bounds .= Bounds.(parse_scalar(T, element, "lower"), upper.(position_bounds))
        end
        if has_attribute(element, "upper")
            position_bounds .= Bounds.(lower.(position_bounds), parse_scalar(T, element, "upper"))
        end
        if has_attribute(element, "velocity")
            v = parse_scalar(T, element, "velocity")
            velocity_bounds .= Bounds(-v, v)
        end
        if has_attribute(element, "effort")
            e = parse_scalar(T, element, "effort")
            effort_bounds .= Bounds(-e, e)
        end
    end
    position_bounds, velocity_bounds, effort_bounds
end

function parse_joint(::Type{T}, xml_joint::XMLElement, floating_joint_type, revolute_joint_type) where {T}
    name = attribute(xml_joint, "name")
    joint_type = parse_joint_type(T, xml_joint, floating_joint_type, revolute_joint_type)
    position_bounds, velocity_bounds, effort_bounds = parse_joint_bounds(joint_type, xml_joint)
    return Joint(name, joint_type; position_bounds=position_bounds, velocity_bounds=velocity_bounds, effort_bounds=effort_bounds)
end

function parse_inertia(::Type{T}, xml_inertial::XMLElement, frame::CartesianFrame3D) where {T}
    urdf_frame = CartesianFrame3D("inertia urdf helper")
    moment = parse_inertia(T, find_element(xml_inertial, "inertia"))
    com = zero(SVector{3, T})
    mass = parse_scalar(T, find_element(xml_inertial, "mass"), "value", "0")
    inertia = SpatialInertia(urdf_frame, moment, com, mass)
    pose = parse_pose(T, find_element(xml_inertial, "origin"))
    transform(inertia, Transform3D(urdf_frame, frame, pose...))
end

function parse_body(::Type{T}, xml_link::XMLElement, frame::CartesianFrame3D = CartesianFrame3D(attribute(xml_link, "name"))) where {T}
    xml_inertial = find_element(xml_link, "inertial")
    inertia = xml_inertial == nothing ? zero(SpatialInertia{T}, frame) : parse_inertia(T, xml_inertial, frame)
    linkname = attribute(xml_link, "name") # TODO: make sure link name is unique
    RigidBody(linkname, inertia)
end

function parse_root_link(mechanism::Mechanism{T}, xml_link::XMLElement, root_joint_type::JointType{T}=Fixed{T}()) where {T}
    parent = root_body(mechanism)
    body = parse_body(T, xml_link)
    joint = Joint("$(string(body))_to_world", root_joint_type)
    joint_to_parent = one(Transform3D{T}, frame_before(joint), default_frame(parent))
    attach!(mechanism, parent, body, joint, joint_pose = joint_to_parent)
end

function parse_joint_and_link(mechanism::Mechanism{T}, xml_parent::XMLElement, xml_child::XMLElement, xml_joint::XMLElement,
        floating_joint_type, revolute_joint_type) where {T}
    parentname = attribute(xml_parent, "name")
    candidate_parents = collect(filter(b -> string(b) == parentname, non_root_bodies(mechanism))) # skip root (name not parsed from URDF)
    length(candidate_parents) == 1 || error("Duplicate name: $(parentname)")
    parent = first(candidate_parents)
    joint = parse_joint(T, xml_joint, floating_joint_type, revolute_joint_type)
    pose = parse_pose(T, find_element(xml_joint, "origin"))
    joint_to_parent = Transform3D(frame_before(joint), default_frame(parent), pose...)
    body = parse_body(T, xml_child, frame_after(joint))
    attach!(mechanism, parent, body, joint, joint_pose = joint_to_parent)
end

"""
$(SIGNATURES)

Create a `Mechanism` by parsing a [URDF](https://wiki.ros.org/urdf/XML/model) file.

Keyword arguments:

* `scalar_type`: the scalar type used to store the `Mechanism`'s kinematic and inertial properties. Default: `Float64`.
* `floating`: whether to use a floating joint as the root joint. Default: false.
* `floating_joint_type`: what `JointType` to use for floating joints. Default: `QuaternionFloating{scalar_type}`.
* `revolute_joint_type`: what `JointType` to use for revolute joints. Default: `Revolute{scalar_type}`.
* `root_joint_type`: the joint type used to connect the parsed `Mechanism` to the world. Default: `floating_joint_type()` if `floating`, `Fixed{scalar_type}()` otherwise.
* `remove_fixed_tree_joints`: whether to remove any fixed joints present in the kinematic tree using [`remove_fixed_tree_joints!`](@ref). Default: `true`.
* `gravity`: gravitational acceleration as a 3-vector expressed in the `Mechanism`'s root frame. Default: `$(DEFAULT_GRAVITATIONAL_ACCELERATION)`.
"""
function parse_urdf(filename::AbstractString;
        scalar_type::Type{T}=Float64,
        floating=false,
        floating_joint_type::Type{<:JointType{T}}=QuaternionFloating{scalar_type},
        revolute_joint_type::Type{<:JointType{T}}=Revolute{scalar_type},
        root_joint_type::JointType{T}=floating ? floating_joint_type() : Fixed{scalar_type}(),
        remove_fixed_tree_joints=true,
        gravity::AbstractVector=DEFAULT_GRAVITATIONAL_ACCELERATION) where T

    if floating && !isfloating(root_joint_type)
        error("Ambiguous input arguments: `floating` specified, but `root_joint_type` is not a floating joint type.")
    end

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
    name_to_vertex = Dict(attribute(v.data, "name") => v for v in vertices)
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
    mechanism = Mechanism(rootbody, gravity=gravity)
    parse_root_link(mechanism, Graphs.root(tree).data, root_joint_type)
    for edge in edges(tree)
        parse_joint_and_link(mechanism, source(edge, tree).data, target(edge, tree).data, edge.data, floating_joint_type, revolute_joint_type)
    end

    if remove_fixed_tree_joints
        remove_fixed_tree_joints!(mechanism)
    end

    mechanism
end

@noinline function parse_urdf(scalar_type::Type, filename::AbstractString)
    # TODO: enable deprecation:
    # replacement = if scalar_type == Float64
    #     "parse_urdf(filename, remove_fixed_tree_joints=false)"
    # else
    #     "parse_urdf(filename, scalar_type=$scalar_type, remove_fixed_tree_joints=false)"
    # end
    # msg = """
    # `parse_urdf(scalar_type, filename)` is deprecated, use $replacement instead.
    # This is to reproduce the exact same behavior as before.
    # You may want to consider leaving `remove_fixed_tree_joints` to its default value (`true`).
    # """
    # Base.depwarn(msg, :parse_urdf)
    parse_urdf(filename; scalar_type=scalar_type, remove_fixed_tree_joints=false)
end
