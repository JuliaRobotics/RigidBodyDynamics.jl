function set_vector_attribute(element::XMLElement, attr::AbstractString, vec::AbstractVector)
    set_attribute(element, attr, join(vec, ' '))
end

function LightXML.XMLElement(body::RigidBody)
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
        if qbound !== realline
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

function LightXML.XMLElement(joint::Joint, mechanism::Mechanism)
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

function LightXML.XMLDocument(mechanism::Mechanism)
    @assert !has_loops(mechanism)

    canonicalized = deepcopy(mechanism)
    canonicalize_graph!(canonicalized)

    xdoc = XMLDocument()
    xroot = create_root(xdoc, "robot")
    for body in bodies(canonicalized)
        add_child(xroot, XMLElement(body))
    end
    for joint in tree_joints(canonicalized)
        add_child(xroot, XMLElement(joint, canonicalized))
    end
    xdoc
end

"""
$(SIGNATURES)

Serialize a `Mechanism` to the [URDF](http://wiki.ros.org/urdf) file format and write it to the stream `io`.
"""
write_urdf(io::IO, mechanism::Mechanism) = show(io, XMLDocument(mechanism))

"""
$(SIGNATURES)

Serialize a `Mechanism` to the [URDF](http://wiki.ros.org/urdf) file format and write it to the file `filename`.
"""
write_urdf(filename::AbstractString, mechanism::Mechanism) = open(io -> write_urdf(io, mechanism), filename, "w")
