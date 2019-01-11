function set_vector_attribute(element::XMLElement, attr::AbstractString, vec::AbstractVector)
    set_attribute(element, attr, join(vec, ' '))
end

function to_urdf(body::RigidBody)
    xml_link = new_element("link")
    set_attribute(xml_link, "name", string(body))
    isroot = !has_defined_inertia(body)
    if !isroot
        xml_inertial = new_child(xml_link, "inertial")
        xml_origin = new_child(xml_inertial, "origin")
        xml_mass = new_child(xml_inertial, "mass")
        xml_inertia = new_child(xml_inertial, "inertia")

        inertia = spatial_inertia(body)
        if inertia.mass > 0
            origin = center_of_mass(inertia)
            centroidal = CartesianFrame3D()
            to_centroidal_frame = Transform3D(inertia.frame, centroidal, -origin.v)
            inertia = transform(inertia, to_centroidal_frame)
            @assert center_of_mass(inertia) ≈ Point3D(centroidal, zero(typeof(origin.v)))
        else
            origin = zero(center_of_mass(inertia))
        end

        set_vector_attribute(xml_origin, "xyz", origin.v)
        set_vector_attribute(xml_origin, "rpy", zero(origin.v))
        set_attribute(xml_mass, "value", inertia.mass)
        set_attribute(xml_inertia, "ixx", inertia.moment[1, 1])
        set_attribute(xml_inertia, "ixy", inertia.moment[1, 2])
        set_attribute(xml_inertia, "ixz", inertia.moment[1, 3])
        set_attribute(xml_inertia, "iyy", inertia.moment[2, 2])
        set_attribute(xml_inertia, "iyz", inertia.moment[2, 3])
        set_attribute(xml_inertia, "izz", inertia.moment[3, 3])
    end
    xml_link
end

function process_joint_type!(xml_joint::XMLElement, joint::Joint)
    if isfloating(joint)
        # handle this here instead of using dispatch to support user-defined
        # floating joint types out of the box
        set_attribute(xml_joint, "type", "floating")
    else
        throw(ArgumentError("Joint type $(typeof(joint_type(joint))) not handled."))
    end
    xml_joint
end

function process_joint_type!(xml_joint::XMLElement, joint::Joint{<:Any, <:Fixed})
    set_attribute(xml_joint, "type", "fixed")
    xml_joint
end

function process_joint_type!(xml_joint::XMLElement, joint::Joint{<:Any, <:Planar})
    set_attribute(xml_joint, "type", "planar")
    jtype = joint_type(joint)
    xml_axis = new_child(xml_joint, "axis")
    set_vector_attribute(xml_axis, "xyz", jtype.x_axis × jtype.y_axis)
    xml_joint
end

function process_joint_type!(xml_joint::XMLElement, joint::Joint{T, JT}) where {T, JT<:Union{Revolute, Prismatic}}
    jtype = joint_type(joint)
    xml_axis = new_child(xml_joint, "axis")
    set_vector_attribute(xml_axis, "xyz", jtype.axis)

    qbounds, vbounds, τbounds = position_bounds(joint), velocity_bounds(joint), effort_bounds(joint)
    @assert length(qbounds) == length(vbounds) == length(τbounds) == 1
    qbound, vbound, τbound = qbounds[1], vbounds[1], τbounds[1]

    @assert upper(vbound) == -lower(vbound)
    @assert upper(τbound) == -lower(τbound)

    realline = Bounds(-typemax(T), typemax(T))
    xml_limit = new_child(xml_joint, "limit")
    set_position_limits = true
    if JT <: Revolute
        if qbound == realline
            set_attribute(xml_joint, "type", "continuous")
            set_position_limits = false # continuous joints don't allow `lower` and `upper`
        else
            set_attribute(xml_joint, "type", "revolute")
        end
    else # Prismatic
        set_attribute(xml_joint, "type", "prismatic")
    end

    if set_position_limits
        set_attribute(xml_limit, "lower", lower(qbound))
        set_attribute(xml_limit, "upper", upper(qbound))
    end

    set_attribute(xml_limit, "effort", upper(τbound))
    set_attribute(xml_limit, "velocity", upper(vbound))

    xml_joint
end

function to_urdf(joint::Joint, mechanism::Mechanism)
    parent = predecessor(joint, mechanism)
    child = successor(joint, mechanism)
    to_parent = joint_to_predecessor(joint)
    xyz = translation(to_parent)
    rpy = RotZYX(rotation(to_parent))

    xml_joint = new_element("joint")
    set_attribute(xml_joint, "name", string(joint))
    xml_parent = new_child(xml_joint, "parent")
    set_attribute(xml_parent, "link", string(parent))
    xml_child = new_child(xml_joint, "child")
    set_attribute(xml_child, "link", string(child))
    xml_origin = new_child(xml_joint, "origin")
    set_vector_attribute(xml_origin, "xyz", xyz)
    set_vector_attribute(xml_origin, "rpy", [rpy.theta3, rpy.theta2, rpy.theta1])

    process_joint_type!(xml_joint, joint)
    xml_joint
end

function to_urdf(mechanism::Mechanism; robot_name::Union{Nothing, AbstractString}=nothing, include_root::Bool=true)
    @assert !has_loops(mechanism)
    xdoc = XMLDocument()
    xroot = create_root(xdoc, "robot")
    if robot_name !== nothing
        set_attribute(xroot, "name", robot_name)
    end
    bodies_to_include = include_root ? bodies(mechanism) : non_root_bodies(mechanism)
    for body in bodies(mechanism)
        !include_root && isroot(body, mechanism) && continue
        add_child(xroot, to_urdf(body))
    end
    for joint in tree_joints(mechanism)
        !include_root && isroot(predecessor(joint, mechanism), mechanism) && continue
        add_child(xroot, to_urdf(joint, mechanism))
    end
    xdoc
end


"""
Serialize a `Mechanism` to the [URDF](https://wiki.ros.org/urdf/XML/model) file format.

Limitations:

* for `<link>` tags, only the `<inertial>` tag is written; there is no support for `<visual>` and `<collision>` tags.
* for `<joint>` tags, only the `<origin>`, `<parent>`, `<child>`, and `<limit>` tags are written. There is no support for the `<calibration>` and `<safety_controller>` tags.

These limitations are simply due to the fact that `Mechanism`s do not store the required information to write these tags.

Keyword arguments:

* `robot_name`: used to set the `name` attribute of the root `<robot>` tag in the URDF. Default: `nothing` (name attribute will not be set).
* `include_root`: whether to include `root_body(mechanism)` in the URDF. If `false`, joints with `root_body(mechanism)` as their predecessor will also be omitted. Default: `true`.
"""
function write_urdf end

const write_urdf_name_kwarg_doc = "Optionally, the `robot_name` keyword argument can be used to specify the robot's name."

"""
$(SIGNATURES)

Write a URDF representation of `mechanism` to the stream `io` (a `Base.IO`).

$write_urdf_name_kwarg_doc
"""
function write_urdf(io::IO, mechanism::Mechanism; robot_name=nothing, include_root=true)
    show(io, to_urdf(mechanism; robot_name=robot_name, include_root=include_root))
end

"""
$(SIGNATURES)

Write a URDF representation of `mechanism` to a file.

$write_urdf_name_kwarg_doc
"""
function write_urdf(filename::AbstractString, mechanism::Mechanism; robot_name=nothing, include_root=true)
    open(filename, "w") do io
        write_urdf(io, mechanism, robot_name=robot_name, include_root=include_root)
    end
end
