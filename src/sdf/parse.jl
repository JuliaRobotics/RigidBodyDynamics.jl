@inline function parse_bool(xml_bool::Union{XMLElement, Nothing}, default::AbstractString)
    parse(Bool, xml_bool === nothing ? default : content(xml_bool))
end

@inline function parse_scalar(::Type{T}, xml_scalar::Union{XMLElement, Nothing}, default::AbstractString) where {T}
    parse(T, xml_scalar === nothing ? default : content(xml_scalar))
end

function parse_pose(::Type{T}, xml_pose::Union{XMLElement, Nothing}) where {T}
    xyzrpy_str = split(xml_pose === nothing ? "0 0 0 0 -0 0" : content(xml_pose))
    @assert length(xyzrpy_str) == 6
    @inbounds x, y, z, roll, pitch, yaw = map(x -> parse(T, x), xyzrpy_str)
    rot = RotMatrix(RotZYX(yaw, pitch, roll))
    trans = SVector(x, y, z)
    xml_frame_name = xml_pose === nothing ? nothing : attribute(xml_pose, "frame")
    frame_name = xml_frame_name === nothing ? nothing : content(xml_frame_name)
    rot, trans, frame_name
end

function parse_inertia_element(::Type{T}, xml_inertia::Union{XMLElement, Nothing}, name::AbstractString, default::AbstractString) where {T}
    parse_scalar(T, xml_inertia === nothing ? nothing: find_element(xml_inertia, name), default)
end

function parse_inertia(::Type{T}, xml_inertia::Union{XMLElement, Nothing}) where {T}
    ixx = parse_inertia_element(T, xml_inertia, "ixx", "1")
    ixy = parse_inertia_element(T, xml_inertia, "ixy", "0")
    ixz = parse_inertia_element(T, xml_inertia, "ixz", "0")
    iyy = parse_inertia_element(T, xml_inertia, "iyy", "1")
    iyz = parse_inertia_element(T, xml_inertia, "iyz", "0")
    izz = parse_inertia_element(T, xml_inertia, "izz", "1")
    @SMatrix [ixx ixy ixz; ixy iyy iyz; ixz iyz izz]
end

function parse_inertial(::Type{T}, xml_inertial::Union{XMLElement, Nothing}, model_frame::CartesianFrame3D) where {T}
    # mass
    xml_mass = xml_inertial === nothing ? nothing : find_element(xml_inertial, "mass")
    mass = parse_scalar(T, xml_mass, "1")

    # inertia
    xml_inertia = xml_inertial === nothing ? nothing : find_element(xml_inertial, "inertia")
    moment = parse_inertia(T, xml_inertial)

    # frame
    if xml_inertial !== nothing
        xml_frames = xml_inertial["frame"]
        # TODO
    end

    # pose
    xml_pose = xml_inertial === nothing ? nothing : find_element(xml_inertial, "pose")
    rot, trans, frame_name = parse_pose(T, xml_pose)
    # TODO: do something with pose

    # TODO: return SpatialInertia
end

function parse_link(::Type{T}, xml_link::XMLElement, model_frame::CartesianFrame3D) where {T}
    # name
    name = attribute(xml_link, "name")
    @assert name != nothing
    body = RigidBody{T}(name)
    body_frame = default_frame(body)

    # TODO: create frame graph
    #

    # gravity
    xml_gravity = find_element(xml_link, "gravity")
    gravity_enabled = parse_bool(xml_gravity, "1")
    @assert gravity_enabled

    # skip wind
    # skip self_collide

    # kinematic
    xml_kinematic = find_element(xml_link, "kinematic")
    kinematic = parse_bool(xml_kinematic, "0")
    @assert !kinematic

    # skip velocity_decay (TODO?)

    # frame
    xml_frames = xml_link["frame"]
    # TODO

    # pose
    xml_pose = find_element(xml_link, "pose")
    rot, trans, frame_name = parse_pose(T, xml_pose)
    # TODO: do something with pose
    # body_transform = Transform3D(body_frame, TODO:frame corresponding to frame_name, rot, trans)

    # inertial
    xml_inertial = find_element(xml_link, "inertial")
    inertia = parse_inertial(T, xml_inertial, name, model_frame)

    # skip collision
    # skip visual
    # skip sensor
    # skip projector
    # skip audio_sink
    # skip audio_source
    # skip battery

    RigidBody(name, inertia)
end

function parse_sdf(filename::AbstractString;
        scalar_type::Type{T}=Float64) where {T}
    # sdf
    xdoc = parse_file(filename)
    xml_sdf = LightXML.root(xdoc)
    @assert LightXML.name(xml_sdf) == "sdf"
    version = VersionNumber(attribute(xml_sdf, "version"))
    @assert version >= v"1.5"

    # TODO: world?, parse gravity.

    # model
    xml_models = xml_sdf["model"]
    @assert length(xml_models) == 1
    xml_model = xml_models[1]
    name = attribute(xml_model, "name")
    @assert name != nothing

    rootbody = RigidBody{T}("world")
    mechanism = Mechanism(rootbody) # TODO: gravity

    # xml_static = find_element(xml_model, "static")
    # static = parse(Bool, xml_static == nothing ? "0" : content(xml_static))
    # @assert !static

    # skip self_collide
    # skip allow_auto_disable

    # includes
    xml_includes = xml_model["include"]
    @assert isempty(xml_includes)

    # model
    xml_submodels = xml_model["model"]
    @assert isempty(xml_submodels)

    # skip enable_wind

    # frame
    xml_frames = xml_model["frame"]
    # TODO

    # pose
    xml_pose = find_element(xml_model, "pose")
    rot, trans, frame_name = parse_pose(T, xml_pose) # TODO: do something with pose

    model_frame = CartesianFrame3D(name)
    model_transform = Transform3D(model_frame, TODO: frame corresponding to frame_name, rot, trans)

    # link
    xml_links = xml_model["link"]
    bodies = map(x -> parse_link(T, x, model_frame), xml_links)

    # joint
    xml_joints = xml_model["joint"]

    # # create graph structure of XML elements
    # graph = DirectedGraph{Vertex{XMLElement}, Edge{XMLElement}}()
    # vertices = Vertex.(xml_links)
    # for vertex in vertices
    #     add_vertex!(graph, vertex)
    # end
    # name_to_vertex = Dict(attribute(v.data, "name") => v for v in vertices)
    # for xml_joint in xml_joints
    #     parent = name_to_vertex[content(find_element(xml_joint, "parent"))]
    #     child = name_to_vertex[content(find_element(xml_joint, "child"))]
    #     add_edge!(graph, parent, child, Edge(xml_joint))
    # end

    # # create a spanning tree
    # roots = collect(filter(v -> isempty(in_edges(v, graph)), vertices))
    # length(roots) != 1 && error("Can only handle a single root")
    # tree = SpanningTree(graph, first(roots))

    # create mechanism from spanning tree




    # skip plugin

    # skip gripper


end
