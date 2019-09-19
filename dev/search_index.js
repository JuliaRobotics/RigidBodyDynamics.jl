var documenterSearchIndex = {"docs": [

{
    "location": "#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "#RigidBodyDynamics-1",
    "page": "Home",
    "title": "RigidBodyDynamics",
    "category": "section",
    "text": "RigidBodyDynamics implements various rigid body dynamics and kinematics algorithms."
},

{
    "location": "#Design-features-1",
    "page": "Home",
    "title": "Design features",
    "category": "section",
    "text": "Some of the key design features of this package are:pure Julia implementation, enabling seamless support for e.g. automatic differentiation using ForwardDiff.jl and symbolic dynamics using SymPy.jl.\neasy creation and modification of general rigid body mechanisms.\nbasic parsing of and writing to the URDF file format.\nextensive checks that verify that coordinate systems match before computation, with the goal of making reference frame mistakes impossible\nflexible caching of intermediate results to prevent doing double work\nfairly small codebase and few dependencies\nsingularity-free rotation parameterizations"
},

{
    "location": "#Functionality-1",
    "page": "Home",
    "title": "Functionality",
    "category": "section",
    "text": "Current functionality of RigidBodyDynamics.jl includes:kinematics/transforming points and free vectors from one coordinate system to another\ntransforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another\nrelative twists/spatial accelerations between bodies\nkinetic/potential energy\ncenter of mass\ngeometric/basic/spatial Jacobians\nmomentum\nmomentum matrix\nmomentum rate bias (= momentum matrix time derivative multiplied by joint velocity vector)\nmass matrix (composite rigid body algorithm)\ninverse dynamics (recursive Newton-Euler)\ndynamics\nsimulation, either using an off-the-shelf ODE integrator or using an included custom Munthe-Kaas integrator that properly handles second-order ODEs defined on a manifold.Closed-loop systems (parallel mechanisms) are supported, with optional Baumgarte stabilization of the loop joint constraints. Support for contact is very limited (possibly subject to major changes in the future), implemented using penalty methods."
},

{
    "location": "#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": ""
},

{
    "location": "#Installing-Julia-1",
    "page": "Home",
    "title": "Installing Julia",
    "category": "section",
    "text": "Download links and more detailed instructions are available on the Julia website. The latest version of RigidBodyDynamics.jl requires Julia 0.7, but we recommend downloading 1.0 (the latest stable Julia release at the time of writing). Version 0.7 of RigidBodyDynamics.jl is the last to support Julia 0.6.warning: Warning\nDo not use apt-get or brew to install Julia, as the versions provided by these package managers tend to be out of date."
},

{
    "location": "#Installing-RigidBodyDynamics-1",
    "page": "Home",
    "title": "Installing RigidBodyDynamics",
    "category": "section",
    "text": "To install the latest tagged release of RigidBodyDynamics, start Julia and enter Pkg mode by pressing ]. Then simply runadd RigidBodyDynamicsTo use the latest master version and work on the bleeding edge (generally, not recommended), instead runadd RigidBodyDynamics#masterA third option is to clone the repository (to the directory printed by julia -e \'import Pkg; println(Pkg.devdir())\'):dev RigidBodyDynamics"
},

{
    "location": "#About-1",
    "page": "Home",
    "title": "About",
    "category": "section",
    "text": "This library was inspired by IHMCRoboticsToolkit and by Drake.Most of the nomenclature used and algorithms implemented by this package stem from the following resources:Murray, Richard M., et al. A mathematical introduction to robotic manipulation. CRC press, 1994.\nFeatherstone, Roy. Rigid body dynamics algorithms. Springer, 2008.\nDuindam, Vincent. Port-based modeling and control for efficient bipedal walking robots. Diss. University of Twente, 2006."
},

{
    "location": "#Contents-1",
    "page": "Home",
    "title": "Contents",
    "category": "section",
    "text": "Pages = [\n  \"spatial.md\",\n  \"joints.md\",\n  \"rigidbody.md\",\n  \"mechanism.md\",\n  \"mechanismstate.md\",\n  \"algorithms.md\",\n  \"caches.md\",\n  \"simulation.md\",\n  \"urdf.md\",\n  \"benchmarks.md\"]\nDepth = 2"
},

{
    "location": "#Citing-this-library-1",
    "page": "Home",
    "title": "Citing this library",
    "category": "section",
    "text": "@misc{rigidbodydynamicsjl,\n author = \"Twan Koolen and contributors\",\n title = \"RigidBodyDynamics.jl\",\n year = 2016,\n url = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl\"\n}"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#",
    "page": "1. Quickstart - double pendulum",
    "title": "1. Quickstart - double pendulum",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/1. Quickstart - double pendulum/1. Quickstart - double pendulum.jl\""
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#.-Quickstart-double-pendulum-1",
    "page": "1. Quickstart - double pendulum",
    "title": "1. Quickstart - double pendulum",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/1. Quickstart - double pendulum\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nend"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#Setup-1",
    "page": "1. Quickstart - double pendulum",
    "title": "Setup",
    "category": "section",
    "text": "In addition to RigidBodyDynamics, we\'ll be using the StaticArrays package, used throughout RigidBodyDynamics, which provides stack-allocated, fixed-size arrays:using RigidBodyDynamics\nusing LinearAlgebra\nusing StaticArrays"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#Creating-a-double-pendulum-Mechanism-1",
    "page": "1. Quickstart - double pendulum",
    "title": "Creating a double pendulum Mechanism",
    "category": "section",
    "text": "We\'re going to create a simple Mechanism that represents a double pendulum. The Mechanism type can be thought of as an interconnection of rigid bodies and joints.We\'ll start by creating a \'root\' rigid body, representing the fixed world, and using it to create a new Mechanism:g = -9.81 # gravitational acceleration in z-direction\nworld = RigidBody{Float64}(\"world\")\ndoublependulum = Mechanism(world; gravity = SVector(0, 0, g))Note that the RigidBody type is parameterized on the \'scalar type\', here Float64.We\'ll now add a second body, called \'upper link\', to the Mechanism. We\'ll attach it to the world with a revolute joint, with the y-axis as the axis of rotation. We\'ll start by creating a SpatialInertia, which stores the inertial properties of the new body:axis = SVector(0., 1., 0.) # joint axis\nI_1 = 0.333 # moment of inertia about joint axis\nc_1 = -0.5 # center of mass location with respect to joint axis\nm_1 = 1. # mass\nframe1 = CartesianFrame3D(\"upper_link\") # the reference frame in which the spatial inertia will be expressed\ninertia1 = SpatialInertia(frame1,\n    moment=I_1 * axis * axis\',\n    com=SVector(0, 0, c_1),\n    mass=m_1)Note that the created SpatialInertia is annotated with the frame in which it is expressed (in the form of a CartesianFrame3D). This is a common theme among RigidBodyDynamics objects. Storing frame information with the data obviates the need for the complicated variable naming conventions that are used in some other libraries to disambiguate the frame in which quantities are expressed. It also enables automated reference frame checks.We\'ll now create the second body:upperlink = RigidBody(inertia1)and a new revolute joint called \'shoulder\':shoulder = Joint(\"shoulder\", Revolute(axis))Creating a Joint automatically constructs two new CartesianFrame3D objects: a frame directly before the joint, and one directly after. To attach the new body to the world by this joint, we\'ll have to specify where the frame before the joint is located on the parent body (here, the world):before_shoulder_to_world = one(Transform3D,\n    frame_before(shoulder), default_frame(world))Now we can attach the upper link to the world:attach!(doublependulum, world, upperlink, shoulder,\n    joint_pose = before_shoulder_to_world)which changes the tree representation of the Mechanism.We can attach the lower link in similar fashion:l_1 = -1. # length of the upper link\nI_2 = 0.333 # moment of inertia about joint axis\nc_2 = -0.5 # center of mass location with respect to joint axis\nm_2 = 1. # mass\ninertia2 = SpatialInertia(CartesianFrame3D(\"lower_link\"),\n    moment=I_2 * axis * axis\',\n    com=SVector(0, 0, c_2),\n    mass=m_2)\nlowerlink = RigidBody(inertia2)\nelbow = Joint(\"elbow\", Revolute(axis))\nbefore_elbow_to_after_shoulder = Transform3D(\n    frame_before(elbow), frame_after(shoulder), SVector(0, 0, l_1))\nattach!(doublependulum, upperlink, lowerlink, elbow,\n    joint_pose = before_elbow_to_after_shoulder)Now our double pendulum Mechanism is complete.Note: instead of defining the Mechanism in this way, it is also possible to load in a URDF file (an XML file format used in ROS), using the parse_urdf function, e.g.:srcdir = dirname(pathof(RigidBodyDynamics))\nurdf = joinpath(srcdir, \"..\", \"test\", \"urdf\", \"Acrobot.urdf\")\nparse_urdf(urdf)"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#The-state-of-a-Mechanism-1",
    "page": "1. Quickstart - double pendulum",
    "title": "The state of a Mechanism",
    "category": "section",
    "text": "A Mechanism stores the joint/rigid body layout, but no state information. State information is separated out into a MechanismState object:state = MechanismState(doublependulum)Let\'s first set the configurations and velocities of the joints:set_configuration!(state, shoulder, 0.3)\nset_configuration!(state, elbow, 0.4)\nset_velocity!(state, shoulder, 1.)\nset_velocity!(state, elbow, 2.);Important: a MechanismState contains cache variables that depend on the configurations and velocities of the joints. These need to be invalidated when the configurations and velocities are changed. To do this, callsetdirty!(state)The joint configurations and velocities are stored as Vectors (denoted q and v respectively in this package) inside the MechanismState object:q = configuration(state)\nv = velocity(state)"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#Kinematics-1",
    "page": "1. Quickstart - double pendulum",
    "title": "Kinematics",
    "category": "section",
    "text": "We are now ready to do kinematics. Here\'s how you transform a point at the origin of the frame after the elbow joint to world frame:transform(state, Point3D(frame_after(elbow), zero(SVector{3})),\n    default_frame(world))Other objects like Wrenches, Twists, and SpatialInertias can be transformed in similar fashion.You can also ask for the homogeneous transform to world:transform_to_root(state, frame_after(elbow))Or a relative transform:relative_transform(state, frame_after(elbow), frame_after(shoulder))and here\'s the center of mass of the double pendulum:center_of_mass(state)"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#Dynamics-1",
    "page": "1. Quickstart - double pendulum",
    "title": "Dynamics",
    "category": "section",
    "text": "A MechanismState can also be used to compute quantities related to the dynamics of the Mechanism. Here we compute the mass matrix:mass_matrix(state)Note that there is also a zero-allocation version, mass_matrix! (the ! at the end of a method is a Julia convention signifying that the function is \'in-place\', i.e. modifies its input data).We can do inverse dynamics as follows (note again that there is a non-allocating version of this method as well):v̇ = similar(velocity(state)) # the joint acceleration vector, i.e., the time derivative of the joint velocity vector v\nv̇[shoulder][1] = 1\nv̇[elbow][1] = 2\ninverse_dynamics(state, v̇)"
},

{
    "location": "generated/1. Quickstart - double pendulum/1. Quickstart - double pendulum/#Simulation-1",
    "page": "1. Quickstart - double pendulum",
    "title": "Simulation",
    "category": "section",
    "text": "Let\'s simulate the double pendulum for 5 seconds, starting from the state we defined earlier. For this, we can use the basic simulate function:ts, qs, vs = simulate(state, 5., Δt = 1e-3);simulate returns a vector of times (ts) and associated joint configurations (qs) and velocities (vs). You can of course plot the trajectories using your favorite plotting package (see e.g. Plots.jl). The MeshCatMechanisms or RigidBodyTreeInspector packages can also be used for 3D animation of the double pendulum in action. See also RigidBodySim.jl for a more full-fledge simulation environment.A lower level interface for simulation/ODE integration with more options is also available. Consult the documentation for more information. In addition, RigidBodySim.jl offers a more full-featured simulation environment.This page was generated using Literate.jl."
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#",
    "page": "2. Closed-loop simulation and visualization",
    "title": "2. Closed-loop simulation and visualization",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization.jl\""
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#.-Closed-loop-simulation-and-visualization-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "2. Closed-loop simulation and visualization",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/2. Closed-loop simulation and visualization\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nendPlease note that RigidBodySim.jl now provides a more capable simulation environment."
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#Setup-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "Setup",
    "category": "section",
    "text": "using RigidBodyDynamics"
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#Model-definition-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "Model definition",
    "category": "section",
    "text": "We\'ll just use the double pendulum model, loaded from a URDF:srcdir = dirname(pathof(RigidBodyDynamics))\nurdf = joinpath(srcdir, \"..\", \"test\", \"urdf\", \"Acrobot.urdf\")\nmechanism = parse_urdf(urdf)"
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#Controller-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "Controller",
    "category": "section",
    "text": "Let\'s write a simple controller that just applies 10 sin(t) at the elbow joint and adds some damping at the shoulder joint:shoulder, elbow = joints(mechanism)\nfunction simple_control!(torques::AbstractVector, t, state::MechanismState)\n    torques[velocity_range(state, shoulder)] .= -1 .* velocity(state, shoulder)\n    torques[velocity_range(state, elbow)] .= 10 * sin(t)\nend;"
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#Simulation-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "Simulation",
    "category": "section",
    "text": "Basic simulation can be done using the simulate function. We\'ll first create a MechanismState object, and set the initial joint configurations and velocities:state = MechanismState(mechanism)\nzero_velocity!(state)\nset_configuration!(state, shoulder, 0.7)\nset_configuration!(state, elbow, -0.8);Now we can simply call simulate, which will return a tuple consisting of:simulation times (a Vector of numbers)\njoint configuration vectors (a Vector of Vectors)\njoint velocity vectors (a Vector of Vectors)final_time = 10.\nts, qs, vs = simulate(state, final_time, simple_control!; Δt = 1e-3);For access to lower-level functionality, such as different ways of storing or visualizing the data generated during the simulation, it is advised to simply pattern match the basic simulate function."
},

{
    "location": "generated/2. Closed-loop simulation and visualization/2. Closed-loop simulation and visualization/#Visualization-1",
    "page": "2. Closed-loop simulation and visualization",
    "title": "Visualization",
    "category": "section",
    "text": "For visualization, we\'ll use MeshCatMechanisms, an external package based on RigidBodyDynamics.jl.using MeshCatMechanismsCreate a MechanismVisualizer and open it in a new browser tab (see MeshCat.jl for other options):mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));# open(mvis)And animate:MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);This page was generated using Literate.jl."
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#",
    "page": "3. Four-bar linkage",
    "title": "3. Four-bar linkage",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/3. Four-bar linkage/3. Four-bar linkage.jl\""
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#.-Four-bar-linkage-1",
    "page": "3. Four-bar linkage",
    "title": "3. Four-bar linkage",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/3. Four-bar linkage\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nendThis example is a (slightly modified) contribution by Aykut Satici."
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#Setup-1",
    "page": "3. Four-bar linkage",
    "title": "Setup",
    "category": "section",
    "text": "using Pkg # hide\nPkg.activate(\"../../../../examples/3. Four-bar linkage\") # hide\nPkg.instantiate() # hide\nusing LinearAlgebra\nusing RigidBodyDynamics\nusing StaticArrays"
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#Model-definition-1",
    "page": "3. Four-bar linkage",
    "title": "Model definition",
    "category": "section",
    "text": "We\'re going to create a four-bar linkage that looks like this: (Image: fourbar)We\'ll \'cut\' the mechanism at joint 4: joints 1, 2, and 3 will be part of the spanning tree of the mechanism, but joint 4 will be a \'loop joint\' (see e.g. Featherstone\'s \'Rigid Body Dynamics Algorithms\'), for which the dynamics will be enforced using Lagrange multipliers.First, we\'ll define some relevant constants:# gravitational acceleration\ng = -9.81\n\n# link lengths\nl_0 = 1.10\nl_1 = 0.5\nl_2 = 1.20\nl_3 = 0.75\n\n# link masses\nm_1 = 0.5\nm_2 = 1.0\nm_3 = 0.75\n\n# link center of mass offsets from the preceding joint axes\nc_1 = 0.25\nc_2 = 0.60\nc_3 = 0.375\n\n# moments of inertia about the center of mass of each link\nI_1 = 0.333\nI_2 = 0.537\nI_3 = 0.4\n\n# Rotation axis: negative y-axis\naxis = SVector(0., -1., 0.);Construct the world rigid body and create a new mechanism:world = RigidBody{Float64}(\"world\")\nfourbar = Mechanism(world; gravity = SVector(0., 0., g))Next, we\'ll construct the spanning tree of the mechanism, consisting of bodies 1, 2, and 3 connected by joints 1, 2, and 3. Note the use of the moment_about_com keyword (as opposed to moment):Link 1 and joint 1:joint1 = Joint(\"joint1\", Revolute(axis))\ninertia1 = SpatialInertia(frame_after(joint1),\n    com=SVector(c_1, 0, 0),\n    moment_about_com=I_1*axis*transpose(axis),\n    mass=m_1)\nlink1 = RigidBody(inertia1)\nbefore_joint1_to_world = one(Transform3D,\n    frame_before(joint1), default_frame(world))\nattach!(fourbar, world, link1, joint1,\n    joint_pose = before_joint1_to_world)Link 2 and joint 2:joint2 = Joint(\"joint2\", Revolute(axis))\ninertia2 = SpatialInertia(frame_after(joint2),\n    com=SVector(c_2, 0, 0),\n    moment_about_com=I_2*axis*transpose(axis),\n    mass=m_2)\nlink2 = RigidBody(inertia2)\nbefore_joint2_to_after_joint1 = Transform3D(\n    frame_before(joint2), frame_after(joint1), SVector(l_1, 0., 0.))\nattach!(fourbar, link1, link2, joint2,\n    joint_pose = before_joint2_to_after_joint1)Link 3 and joint 3:joint3 = Joint(\"joint3\", Revolute(axis))\ninertia3 = SpatialInertia(frame_after(joint3),\n    com=SVector(l_0, 0., 0.),\n    moment_about_com=I_3*axis*transpose(axis),\n    mass=m_3)\nlink3 = RigidBody(inertia3)\nbefore_joint3_to_world = Transform3D(\n    frame_before(joint3), default_frame(world), SVector(l_0, 0., 0.))\nattach!(fourbar, world, link3, joint3, joint_pose = before_joint3_to_world)Finally, we\'ll add joint 4 in almost the same way we did the other joints, with the following exceptions:both link2 and link3 are already part of the Mechanism, so the attach! function will figure out that joint4 will be a loop joint.\ninstead of using the default (identity) for the argument that specifies the transform from the successor of joint 4 (i.e., link 3) to the frame directly afterjoint 4, we\'ll specify a transform that incorporates the l_3 offset.# joint between link2 and link3\njoint4 = Joint(\"joint4\", Revolute(axis))\nbefore_joint4_to_joint2 = Transform3D(\n    frame_before(joint4), frame_after(joint2), SVector(l_2, 0., 0.))\njoint3_to_after_joint4 = Transform3D(\n    frame_after(joint3), frame_after(joint4), SVector(-l_3, 0., 0.))\nattach!(fourbar, link2, link3, joint4,\n    joint_pose = before_joint4_to_joint2, successor_pose = joint3_to_after_joint4)Note the additional non-tree joint in the printed Mechanism summary."
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#Simulation-1",
    "page": "3. Four-bar linkage",
    "title": "Simulation",
    "category": "section",
    "text": "As usual, we\'ll now construct a MechanismState and DynamicsResult for the four-bar Mechanism. We\'ll set some initial conditions for a simulation, which were solved for a priori using a nonlinear program (not shown here).state = MechanismState(fourbar)\nresult = DynamicsResult(fourbar);\n\nset_configuration!(state, joint1, 1.6707963267948966) # θ\nset_configuration!(state, joint2, -1.4591054166649482) # γ\nset_configuration!(state, joint3, 1.5397303602625536) # ϕ\n\nset_velocity!(state, joint1, 0.5)\nset_velocity!(state, joint2, -0.47295)\nset_velocity!(state, joint3, 0.341)\n\n# Invalidate the cache variables\nsetdirty!(state)Next, we\'ll do a 3-second simulation:ts, qs, vs = simulate(state, 3., Δt = 1e-2);"
},

{
    "location": "generated/3. Four-bar linkage/3. Four-bar linkage/#Visualization-1",
    "page": "3. Four-bar linkage",
    "title": "Visualization",
    "category": "section",
    "text": "For visualization, we\'ll use MeshCatMechanisms, an external package based on RigidBodyDynamics.jl.using MeshCatMechanismsCreate a MechanismVisualizer for the four-bar linkage and open it in a new browser tab (see MeshCat.jl for other options):mvis = MechanismVisualizer(fourbar, Skeleton(inertias=false));# open(mvis)And animate:MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);This page was generated using Literate.jl."
},

{
    "location": "generated/4. Jacobian IK and Control/4. Jacobian IK and Control/#",
    "page": "4. Jacobian IK and Control",
    "title": "4. Jacobian IK and Control",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/4. Jacobian IK and Control/4. Jacobian IK and Control.jl\""
},

{
    "location": "generated/4. Jacobian IK and Control/4. Jacobian IK and Control/#.-Jacobian-IK-and-Control-1",
    "page": "4. Jacobian IK and Control",
    "title": "4. Jacobian IK and Control",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/4. Jacobian IK and Control\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nendIn this notebook, we\'ll demonstrate an extremely simple approach for computing basic inverse kinematics (IK) and controlling the position of some point on our robot using the Jacobian transpose.For a brief technical introduction, see https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf or https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf"
},

{
    "location": "generated/4. Jacobian IK and Control/4. Jacobian IK and Control/#Setup-1",
    "page": "4. Jacobian IK and Control",
    "title": "Setup",
    "category": "section",
    "text": "using Pkg # hide\nPkg.activate(\"../../../../examples/4. Jacobian IK and Control\") # hide\nPkg.instantiate() # hide\nusing RigidBodyDynamics\nusing StaticArrays\nusing MeshCatMechanisms, BlinkFix the random seed, so we get repeatable resultsusing Random\nRandom.seed!(42);First we\'ll load our double pendulum robot from URDF:srcdir = dirname(pathof(RigidBodyDynamics))\nurdf = joinpath(srcdir, \"..\", \"test\", \"urdf\", \"Acrobot.urdf\")\nmechanism = parse_urdf(urdf)\nstate = MechanismState(mechanism)\nmechanismNow we choose a point on the robot to control. We\'ll pick the end of the second link, which is located 2m from the origin of the lower_link body:body = findbody(mechanism, \"lower_link\")\npoint = Point3D(default_frame(body), 0., 0, -2)Let\'s visualize the mechanism and its attached point. For visualization, we\'ll use MeshCatMechanisms.jl with Blink.jl.# Create the visualizer\nvis = MechanismVisualizer(mechanism, URDFVisuals(urdf))\n\n# Render our target point attached to the robot as a sphere with radius 0.07\nsetelement!(vis, point, 0.07)Open the visualizer in a new Blink window:# open(vis, Window());Create a MechanismVisualizer:mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));"
},

{
    "location": "generated/4. Jacobian IK and Control/4. Jacobian IK and Control/#Inverse-Kinematics-1",
    "page": "4. Jacobian IK and Control",
    "title": "Inverse Kinematics",
    "category": "section",
    "text": "First, let\'s use the point jacobian to solve a simple inverse kinematics problem. Given a target location desired expressed in world frame, we want to find the joint angles q such that the point attached to the robot is at the desired location.To do that, we\'ll iteratively update q by applying:\\begin{align} \\Delta q = \\alpha \\, J_p^\\top \\, \\Delta p \\end{align}where alpha is our step size (equivalent to a learning rate in gradient descent) and Delta p is the error in the position of our target point.function jacobian_transpose_ik!(state::MechanismState,\n                               body::RigidBody,\n                               point::Point3D,\n                               desired::Point3D;\n                               α=0.1,\n                               iterations=100)\n    mechanism = state.mechanism\n    world = root_frame(mechanism)\n\n    # Compute the joint path from world to our target body\n    p = path(mechanism, root_body(mechanism), body)\n    # Allocate the point jacobian (we\'ll update this in-place later)\n    Jp = point_jacobian(state, p, transform(state, point, world))\n\n    q = copy(configuration(state))\n\n    for i in 1:iterations\n        # Update the position of the point\n        point_in_world = transform(state, point, world)\n        # Update the point\'s jacobian\n        point_jacobian!(Jp, state, p, point_in_world)\n        # Compute an update in joint coordinates using the jacobian transpose\n        Δq = α * Array(Jp)\' * (transform(state, desired, world) - point_in_world).v\n        # Apply the update\n        q .= configuration(state) .+ Δq\n        set_configuration!(state, q)\n    end\n    state\nendTo use our IK method, we just have to set our current state and choose a desired location for the tip of the robot\'s arm:rand!(state)\nset_configuration!(vis, configuration(state))Choose a desired location. We\'ll move the tip of the arm to [0.5, 0, 2]desired_tip_location = Point3D(root_frame(mechanism), 0.5, 0, 2)Run the IK, updating state in placejacobian_transpose_ik!(state, body, point, desired_tip_location)\nset_configuration!(vis, configuration(state))We asked for our point to be close to [0.5, 0, 2], but since the arm cannot move in the y direction at all we end up near [0.5, 0.25, 2] insteadtransform(state, point, root_frame(mechanism))We can try varying the target and watching the IK solution change:qs = typeof(configuration(state))[]Vary the desired x position from -1 to 1for x in range(-1, stop=1, length=100)\n    desired = Point3D(root_frame(mechanism), x, 0, 2)\n    jacobian_transpose_ik!(state, body, point, desired)\n    push!(qs, copy(configuration(state)))\nend\nts = collect(range(0, stop=1, length=length(qs)))\nsetanimation!(vis, ts, qs)"
},

{
    "location": "generated/4. Jacobian IK and Control/4. Jacobian IK and Control/#Control-1",
    "page": "4. Jacobian IK and Control",
    "title": "Control",
    "category": "section",
    "text": "Now let\'s use the same principle to generate torques and actually control the robot. To make things more interesting, let\'s get the end of the robot\'s arm to trace out a circle.circle_origin = SVector(0., 0.25, 2)\nradius = 0.5\nω = 1.0  # radians per second at which the point should move in its circle\n\nusing MeshCat\nusing GeometryTypes: Point\n\n# Draw the circle in the viewer\nθ = repeat(range(0, stop=2π, length=100), inner=(2,))[2:end]\ncx, cy, cz = circle_origin\ngeometry = PointCloud(Point.(cx .+ radius .* sin.(θ), cy, cz .+ 0.5 .* cos.(θ)))\nsetobject!(vis[:circle], LineSegments(geometry, LineBasicMaterial()))This function will take in the parameters of the circle and the target point and return a function we can use as the controller. By wrapping the construction of the controller in this way, we avoid any issues with accessing non-const global variables.function make_circle_controller(state::MechanismState,\n                                body::RigidBody,\n                                point::Point3D,\n                                circle_origin::AbstractVector,\n                                radius,\n                                ω)\n    mechanism = state.mechanism\n    world = root_frame(mechanism)\n    joint_path = path(mechanism, root_body(mechanism), body)\n    point_world = transform(state, point, root_frame(mechanism))\n    Jp = point_jacobian(state, joint_path, point_world)\n    v̇ = similar(velocity(state))\n\n    function controller!(τ, t, state)\n        desired = Point3D(world, circle_origin .+ radius .* SVector(sin(t / ω), 0, cos(t / ω)))\n        point_in_world = transform_to_root(state, body) * point\n        point_jacobian!(Jp, state, joint_path, point_in_world)\n        Kp = 200\n        Kd = 20\n        Δp = desired - point_in_world\n        v̇ .= Kp * Array(Jp)\' * Δp.v .- 20 .* velocity(state)\n        τ .= inverse_dynamics(state, v̇)\n    end\nend\ncontroller! = make_circle_controller(state, body, point, circle_origin, radius, ω)\nts, qs, vs = simulate(state, 10, controller!);Animate the resulting trajectory:setanimation!(vis, ts, qs)Now we can plot the behavior of the controller. The initial state is quite far from the target, so there\'s some significant overshoot early in the trajectory, but the controller eventually settles into tracking the desired circular path. This controller isn\'t very well-tuned, and we could certainly do better with a more advanced approach, but this is still a nice demonstration of a very simple control policy.using Plots; gr()\n\nxs = Float64[]\nzs = Float64[]\n\n# Downsample by 100 just so the plot doesn\'t become a huge file:\nfor q in qs[1:100:end]\n    set_configuration!(state, q)\n    p = transform(state, point, root_frame(mechanism))\n    push!(xs, p.v[1])\n    push!(zs, p.v[3])\nend\n\nplot(xs, zs, xlim=(-1, 1), ylim=(1, 3), aspect_ratio=:equal, legend=nothing)This page was generated using Literate.jl."
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "5. Derivatives and gradients using ForwardDiff",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff.jl\""
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#.-Derivatives-and-gradients-using-ForwardDiff-1",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "5. Derivatives and gradients using ForwardDiff",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/5. Derivatives and gradients using ForwardDiff\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nend"
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#Setup-1",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "Setup",
    "category": "section",
    "text": "using Pkg # hide\nPkg.activate(\"../../../../examples/5. Derivatives and gradients using ForwardDiff\") # hide\nPkg.instantiate() # hide\nusing RigidBodyDynamics, StaticArrays, ForwardDiff\nusing Test, Random\nRandom.seed!(1); # to get repeatable results"
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#Jacobians-with-respect-to-q-and-v-the-naive-way-1",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "Jacobians with respect to q and v - the naive way",
    "category": "section",
    "text": "First, we\'ll load our trusty double pendulum from a URDF:srcdir = dirname(pathof(RigidBodyDynamics))\nurdf = joinpath(srcdir, \"..\", \"test\", \"urdf\", \"Acrobot.urdf\")\nmechanism = parse_urdf(urdf)Of course, we can create a MechanismState for the double pendulum, and compute its momentum in some random state:float64state = MechanismState(mechanism)\nrand!(float64state)\nmomentum(float64state)But now suppose we want the Jacobian of momentum with respect to the joint velocity vector v. We can do this using the ForwardDiff.Dual type and the ForwardDiff.jacobian function. The ForwardDiff package implements forward-mode automatic differentiation.To use ForwardDiff.jacobian we\'ll create a function that maps v (as a Vector) to momentum (as a Vector):q = configuration(float64state)\nfunction momentum_vec(v::AbstractVector{T}) where T\n    # create a `MechanismState` that can handle the element type of `v` (which will be some `ForwardDiff.Dual`):\n    state = MechanismState{T}(mechanism)\n\n    # set the state variables:\n    set_configuration!(state, q)\n    set_velocity!(state, v)\n\n    # return momentum converted to an `SVector` (as ForwardDiff expects an `AbstractVector`)\n    Vector(SVector(momentum(state)))\nendLet\'s first check that the function returns the same thing we got from float64state:v = velocity(float64state)\n@test momentum_vec(v) == SVector(momentum(float64state))That works, so now let\'s compute the Jacobian with ForwardDiff:J = ForwardDiff.jacobian(momentum_vec, v)At this point we note that the matrix J is simply the momentum matrix (in world frame) of the Mechanism. In this case, RigidBodyDynamics.jl has a specialized algorithm for computing this matrix, so let\'s verify the results:A = momentum_matrix(float64state)\n@test J ≈ Array(A) atol = 1e-12Gradients with respect to q can be computed in similar fashion."
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#Improving-performance-1",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "Improving performance",
    "category": "section",
    "text": "Ignoring the fact that we have a specialized method available, let\'s look at the performance of using ForwardDiff.jacobian.using BenchmarkTools\n@benchmark ForwardDiff.jacobian($momentum_vec, $v)That\'s not great. Note all the allocations. We can do better by making the following modifications:use an in-place version of the jacobian function, ForwardDiff.jacobian!\nreimplement our momentum_vec function to be in-place as well\ndon\'t create a new MechanismState every timeThe third point is especially important; creating a MechanismState is expensive!Regarding the second point, we could also just stop converting momentum from a StaticArrays.SVector to a Vector to avoid allocations. However, the solution of making the function in-place also applies when the size of the output vector is not known statically (e.g., for dynamics_bias!).To facillitate reuse of MechanismStates while keeping the code nice and generic, we can use a StateCache object. StateCache is a container that stores MechanismStates of various types (associated with one Mechanism), and will ease the process of using ForwardDiff. Creating one is easy:const statecache = StateCache(mechanism)MechanismStates of a given type can be accessed as follows (note that if a MechanismState of a certain type is already available, it will be reused):float32state = statecache[Float32]\n@test float32state === statecache[Float32]Now we\'ll use the StateCache to reimplement momentum_vec, making it in-place as well:function momentum_vec!(out::AbstractVector, v::AbstractVector{T}) where T\n    # retrieve a `MechanismState` that can handle the element type of `v`:\n    state = statecache[T]\n\n    # set the state variables:\n    set_configuration!(state, q)\n    set_velocity!(state, v)\n\n    # compute momentum and store it in `out`\n    m = momentum(state)\n    copyto!(out, SVector(m))\nendCheck that the in-place version works as expected on Float64 inputs:const out = zeros(6) # where we\'ll be storing our results\nmomentum_vec!(out, v)\n@test out == SVector(momentum(float64state))And use ForwardDiff.jacobian! to compute the Jacobian:const result = DiffResults.JacobianResult(out, v)\nconst config = ForwardDiff.JacobianConfig(momentum_vec!, out, v)\nForwardDiff.jacobian!(result, momentum_vec!, out, v, config)\nJ = DiffResults.jacobian(result)\n@test J ≈ Array(A) atol = 1e-12Let\'s check the performance again:@benchmark ForwardDiff.jacobian!($result, $momentum_vec!, $out, $v, $config)That\'s much better. Do note that the specialized algorithm is still faster:q = copy(configuration(float64state))\n@benchmark begin\n    set_configuration!($float64state, $q)\n    momentum_matrix!($A, $float64state)\nend"
},

{
    "location": "generated/5. Derivatives and gradients using ForwardDiff/5. Derivatives and gradients using ForwardDiff/#Time-derivatives-1",
    "page": "5. Derivatives and gradients using ForwardDiff",
    "title": "Time derivatives",
    "category": "section",
    "text": "We can also use ForwardDiff to compute time derivatives. Let\'s verify that energy is conserved for the double pendulum in the absence of nonconservative forces (like damping). That is, we expect that the time derivative of the pendulum\'s total energy is zero when its state evolves according to the passive dynamics.Let\'s first compute the joint acceleration vector dotv using the passive dynamics:dynamicsresult = DynamicsResult(mechanism)\nset_configuration!(float64state, q)\nset_velocity!(float64state, v)\ndynamics!(dynamicsresult, float64state)\nv̇ = dynamicsresult.v̇Now for the time derivative of total energy. ForwardDiff has a derivative function that can be used to take derivatives of functions that map a scalar to a scalar. But in this example, we\'ll instead use ForwardDiff\'s Dual type directly. ForwardDiff.Dual represents a (potentially multidimensional) dual number, i.e., a type that stores both the value of a function evaluated at a certain point, as well as the partial derivatives of the function, again evaluated at the same point. See the ForwardDiff documentation for more information.We\'ll create a vector of Duals representing the value and derivative of q(t):q̇ = v\nq_dual = ForwardDiff.Dual.(q, q̇)Note: for the double pendulum, dotq = v, but this is not the case in general for Mechanisms created using RigidBodyDynamics.jl. For example, the QuaternionSpherical joint type uses a unit quaternion to represent the joint configuration, but angular velocity (in body frame) to represent velocity. In general dotq can be computed from the velocity vector v stored in a MechanismState usingconfiguration_derivative(::MechanismState)or its in-place variant, configuration_derivative!.We\'ll do the same thing for v(t):v_dual = ForwardDiff.Dual.(v, v̇)Now we\'re ready to compute the total energy (kinetic + potential) using these ForwardDiff.Dual inputs. We\'ll use our StateCache again:T = eltype(q_dual)\nstate = statecache[T]\nset_configuration!(state, q_dual)\nset_velocity!(state, v_dual)\nenergy_dual = kinetic_energy(state) + gravitational_potential_energy(state)Note that the result type of energy_dual is again a ForwardDiff.Dual. We can extract the energy and its time derivative (mechanical power) from energy_dual as follows:energy = ForwardDiff.value(energy_dual)\npartials = ForwardDiff.partials(energy_dual)\npower = partials[1];So the total energy in the system is:energyNote: the total energy is negative because the origin of the world frame is used as a reference for computing gravitational potential energy, i.e., the center of mass of the double pendulum is somewhere below this origin.And we can verify that, indeed, there is no power flow into or out of the system:@test power ≈ 0 atol = 1e-14This page was generated using Literate.jl."
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#",
    "page": "6. Symbolics using SymPy",
    "title": "6. Symbolics using SymPy",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/6. Symbolics using SymPy/6. Symbolics using SymPy.jl\""
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#.-Symbolics-using-SymPy-1",
    "page": "6. Symbolics using SymPy",
    "title": "6. Symbolics using SymPy",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/6. Symbolics using SymPy\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nend"
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#Setup-1",
    "page": "6. Symbolics using SymPy",
    "title": "Setup",
    "category": "section",
    "text": "using Pkg # hide\nPkg.activate(\"../../../../examples/6. Symbolics using SymPy\") # hide\nPkg.instantiate() # hide\nusing RigidBodyDynamics\nusing StaticArrays\nusing SymPy"
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#Create-symbolic-parameters-1",
    "page": "6. Symbolics using SymPy",
    "title": "Create symbolic parameters",
    "category": "section",
    "text": "Masses: m_1 m_2\nMass moments of inertia (about center of mass): I_1 I_2\nLink lengths: l_1 l_2\nCenter of mass locations (w.r.t. preceding joint axis): c_1 c_2\nGravitational acceleration: ginertias = @syms m_1 m_2 I_1 I_2 positive = true\nlengths = @syms l_1 l_2 c_1 c_2 real = true\ngravitational_acceleration = @syms g real = true\nparams = [inertias..., lengths..., gravitational_acceleration...]\ntranspose(params)"
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#Create-double-pendulum-Mechanism-1",
    "page": "6. Symbolics using SymPy",
    "title": "Create double pendulum Mechanism",
    "category": "section",
    "text": "A Mechanism contains the joint layout and inertia parameters, but no state information.T = Sym # the \'scalar type\' of the Mechanism we\'ll construct\naxis = SVector(zero(T), one(T), zero(T)) # axis of rotation for each of the joints\ndouble_pendulum = Mechanism(RigidBody{T}(\"world\"); gravity = SVector(zero(T), zero(T), g))\nworld = root_body(double_pendulum) # the fixed \'world\' rigid bodyAttach the first (upper) link to the world via a revolute joint named \'shoulder\'inertia1 = SpatialInertia(CartesianFrame3D(\"upper_link\"),\n    moment=I_1 * axis * transpose(axis),\n    com=SVector(zero(T), zero(T), c_1),\n    mass=m_1)\nbody1 = RigidBody(inertia1)\njoint1 = Joint(\"shoulder\", Revolute(axis))\njoint1_to_world = one(Transform3D{T}, frame_before(joint1), default_frame(world));\nattach!(double_pendulum, world, body1, joint1,\n    joint_pose = joint1_to_world);Attach the second (lower) link to the world via a revolute joint named \'elbow\'inertia2 = SpatialInertia(CartesianFrame3D(\"lower_link\"),\n    moment=I_2 * axis * transpose(axis),\n    com=SVector(zero(T), zero(T), c_2),\n    mass=m_2)\nbody2 = RigidBody(inertia2)\njoint2 = Joint(\"elbow\", Revolute(axis))\njoint2_to_body1 = Transform3D(\n    frame_before(joint2), default_frame(body1), SVector(zero(T), zero(T), l_1))\nattach!(double_pendulum, body1, body2, joint2,\n    joint_pose = joint2_to_body1)"
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#Create-MechanismState-associated-with-the-double-pendulum-Mechanism-1",
    "page": "6. Symbolics using SymPy",
    "title": "Create MechanismState associated with the double pendulum Mechanism",
    "category": "section",
    "text": "A MechanismState stores all state-dependent information associated with a Mechanism.x = MechanismState(double_pendulum);Set the joint configuration vector of the MechanismState to a new vector of symbolic variablesq = configuration(x)\nfor i in eachindex(q)\n    q[i] = symbols(\"q_$i\", real = true)\nendSet the joint velocity vector of the MechanismState to a new vector of symbolic variablesv = velocity(x)\nfor i in eachindex(v)\n    v[i] = symbols(\"v_$i\", real = true)\nend"
},

{
    "location": "generated/6. Symbolics using SymPy/6. Symbolics using SymPy/#Compute-dynamical-quantities-in-symbolic-form-1",
    "page": "6. Symbolics using SymPy",
    "title": "Compute dynamical quantities in symbolic form",
    "category": "section",
    "text": "Mass matrixsimplify.(mass_matrix(x))Kinetic energysimplify(kinetic_energy(x))Potential energysimplify(gravitational_potential_energy(x))This page was generated using Literate.jl."
},

{
    "location": "generated/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic/#",
    "page": "7. Rigorous error bounds using IntervalArithmetic",
    "title": "7. Rigorous error bounds using IntervalArithmetic",
    "category": "page",
    "text": "EditURL = \"https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/examples/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic.jl\""
},

{
    "location": "generated/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic/#.-Rigorous-error-bounds-using-IntervalArithmetic-1",
    "page": "7. Rigorous error bounds using IntervalArithmetic",
    "title": "7. Rigorous error bounds using IntervalArithmetic",
    "category": "section",
    "text": "This example is also available as a Jupyter notebook that can be run locally The notebook can be found in the examples directory of the package. If the notebooks are missing, you may need to run using Pkg; Pkg.build().import Pkg\nlet\n    original_stdout = stdout\n    out_rd, out_wr = redirect_stdout()\n    @async read(out_rd, String)\n    try\n        Pkg.activate(\"../../../../examples/7. Rigorous error bounds using IntervalArithmetic\")\n        Pkg.instantiate()\n    finally\n        redirect_stdout(original_stdout)\n        close(out_wr)\n    end\nend"
},

{
    "location": "generated/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic/#Floating-point-error-1",
    "page": "7. Rigorous error bounds using IntervalArithmetic",
    "title": "Floating-point error",
    "category": "section",
    "text": "In computers, real numbers are commonly approximated using floating-point numbers, such as Julia\'s Float64. Unfortunately, not all real numbers can be exactly represented as a finite-size floating-point number, and the results of operations on floating-point numbers can only approximate the results of applying the operation to a true real number. This results in peculiarities like:2.6 - 0.7 - 1.9IntervalArithmetic.jl can be used to quantify floating point error, by computing rigorous worst-case bounds on floating point error, within which the true result is guaranteed to lie.using Pkg # hide\nPkg.activate(\"../../../../examples/7. Rigorous error bounds using IntervalArithmetic\") # hide\nPkg.instantiate() # hide\nusing IntervalArithmeticIntervalArithmetic.jl provides the Interval type, which stores an upper and a lower bound:i = Interval(1.0, 2.0)dump(i)IntervalArithmetic.jl provides overloads for most common Julia functions that take these bounds into account. For example:i + isin(i)Note that the bounds computed by IntervalArithmetic.jl take floating point error into account. Also note that a given real number, once converted to (approximated by) a floating-point number may not be equal to the original real number. To rigorously construct an Interval that contains a given real number as an input, IntervalArithmetic.jl provides the @interval macro:i = @interval(2.9)\ni.lo === i.hidump(i)Compare this toi = Interval(2.9)\ni.lo === i.hidump(i)As an example, consider again the peculiar result from before, now using interval arithmetic:i = @interval(2.6) - @interval(0.7) - @interval(1.9)showing that the true result, 0, is indeed in the guaranteed interval, and indeed:using Test\n@test (2.6 - 0.7 - 1.9) ∈ i"
},

{
    "location": "generated/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic/#Accuracy-of-RigidBodyDynamics.jl\'s-mass_matrix-1",
    "page": "7. Rigorous error bounds using IntervalArithmetic",
    "title": "Accuracy of RigidBodyDynamics.jl\'s mass_matrix",
    "category": "section",
    "text": "Let\'s use IntervalArithmetic.jl to establish rigorous bounds on the accuracy of the accuracy of the mass_matrix algorithm for the Acrobot (double pendulum) in a certain configuration. Let\'s get started.using RigidBodyDynamicsWe\'ll create a Mechanism by parsing the Acrobot URDF, passing in Interval{Float64} as the type used to store the parameters (inertias, link lengths, etc.) of the mechanism. Note that the parameters parsed from the URDF are treated as floating point numbers (i.e., like Interval(2.9) instead of @interval(2.9) above).const T = Interval{Float64}\nsrcdir = dirname(pathof(RigidBodyDynamics))\nurdf = joinpath(srcdir, \"..\", \"test\", \"urdf\", \"Acrobot.urdf\")\nconst mechanism = parse_urdf(urdf; scalar_type=T)\nstate = MechanismState(mechanism)Let\'s set the initial joint angle of the shoulder joint to the smallest Interval{Float64} containing the real number 1, and similarly for the elbow joint:shoulder, elbow = joints(mechanism)\nset_configuration!(state, shoulder, @interval(1))\nset_configuration!(state, elbow, @interval(2));And now we can compute the mass matrix as normal:M = mass_matrix(state)Woah, those bounds look pretty big. RigidBodyDynamics.jl must not be very accurate! Actually, things aren\'t so bad; the issue is just that IntervalArithmetic.jl isn\'t kidding when it comes to guaranteed bounds, and that includes printing the numbers in shortened form. Here are the lengths of the intervals:err = map(x -> x.hi - x.lo, M)@test maximum(abs, err) ≈ 0 atol = 1e-14"
},

{
    "location": "generated/7. Rigorous error bounds using IntervalArithmetic/7. Rigorous error bounds using IntervalArithmetic/#Rigorous-(worst-case)-uncertainty-propagation-1",
    "page": "7. Rigorous error bounds using IntervalArithmetic",
    "title": "Rigorous (worst-case) uncertainty propagation",
    "category": "section",
    "text": "IntervalArithmetic.jl can also be applied to propagate uncertainty in a rigorous way when the inputs themselves are uncertain. Consider for example the case that we only know the joint angles up to pm 005 radians:set_configuration!(state, shoulder, @interval(0.95, 1.05))\nset_configuration!(state, elbow, @interval(1.95, 2.05));and let\'s compute bounds on the center of mass position:center_of_mass(state)Note that the bounds on the y-coordinate are very tight, since our mechanism only lives in the x-z plane.This page was generated using Literate.jl."
},

{
    "location": "spatial/#",
    "page": "Spatial vector algebra",
    "title": "Spatial vector algebra",
    "category": "page",
    "text": ""
},

{
    "location": "spatial/#Spatial-vector-algebra-1",
    "page": "Spatial vector algebra",
    "title": "Spatial vector algebra",
    "category": "section",
    "text": ""
},

{
    "location": "spatial/#Index-1",
    "page": "Spatial vector algebra",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"spatial.md\"]\nOrder   = [:type, :function, :macro]"
},

{
    "location": "spatial/#Types-1",
    "page": "Spatial vector algebra",
    "title": "Types",
    "category": "section",
    "text": ""
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.CartesianFrame3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.CartesianFrame3D",
    "category": "type",
    "text": "struct CartesianFrame3D\n\nA CartesianFrame3D identifies a three-dimensional Cartesian coordinate system.\n\nCartesianFrame3Ds are typically used to annotate the frame in which certain quantities are expressed.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.CartesianFrame3D-Tuple{String}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.CartesianFrame3D",
    "category": "method",
    "text": "CartesianFrame3D(name)\n\n\nCreate a CartesianFrame3D with the given name.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.CartesianFrame3D-Tuple{}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.CartesianFrame3D",
    "category": "method",
    "text": "CartesianFrame3D()\n\n\nCreate an anonymous CartesianFrame3D.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Coordinate-frames-1",
    "page": "Spatial vector algebra",
    "title": "Coordinate frames",
    "category": "section",
    "text": "CartesianFrame3D\nCartesianFrame3D(::String)\nCartesianFrame3D()"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.Transform3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.Transform3D",
    "category": "type",
    "text": "struct Transform3D{T}\n\nA homogeneous transformation matrix representing the transformation from one three-dimensional Cartesian coordinate system to another.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Transforms-1",
    "page": "Spatial vector algebra",
    "title": "Transforms",
    "category": "section",
    "text": "Transform3D"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.Point3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.Point3D",
    "category": "type",
    "text": "struct Point3D{V<:(AbstractArray{T,1} where T)}\n\nA Point3D represents a position in a given coordinate system.\n\nA Point3D is a bound vector. Applying a Transform3D to a Point3D both rotates and translates the Point3D.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.FreeVector3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.FreeVector3D",
    "category": "type",
    "text": "struct FreeVector3D{V<:(AbstractArray{T,1} where T)}\n\nA FreeVector3D represents a free vector.\n\nExamples of free vectors include displacements and velocities of points.\n\nApplying a Transform3D to a FreeVector3D only rotates the FreeVector3D.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Points,-free-vectors-1",
    "page": "Spatial vector algebra",
    "title": "Points, free vectors",
    "category": "section",
    "text": "Point3D\nFreeVector3D"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.SpatialInertia",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.SpatialInertia",
    "category": "type",
    "text": "struct SpatialInertia{T}\n\nA spatial inertia, or inertia matrix, represents the mass distribution of a rigid body.\n\nA spatial inertia expressed in frame i is defined as:\n\nI^i =\nint_Brholeft(xright)leftbeginarraycc\nhatp^Tleft(xright)hatpleft(xright)  hatpleft(xright)\nhatp^Tleft(xright)  I\nendarrayrightdx=leftbeginarraycc\nJ  hatc\nhatc^T  mI\nendarrayright\n\nwhere rho(x) is the density of point x, and p(x) are the coordinates of point x expressed in frame i. J is the mass moment of inertia, m is the total mass, and c is the \'cross part\', center of mass position scaled by m.\n\nwarning: Warning\nThe moment field of a SpatialInertia is the moment of inertia about the origin of its frame, not about the center of mass.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Inertias-1",
    "page": "Spatial vector algebra",
    "title": "Inertias",
    "category": "section",
    "text": "SpatialInertia"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.Twist",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.Twist",
    "category": "type",
    "text": "struct Twist{T}\n\nA twist represents the relative angular and linear motion between two bodies.\n\nThe twist of frame j with respect to frame i, expressed in frame k is defined as\n\nT_j^ki=left(beginarrayc\nomega_j^ki\nv_j^ki\nendarrayright)inmathbbR^6\n\nsuch that\n\nleftbeginarraycc\nhatomega_j^ki  v_j^ki\n0  0\nendarrayright=H_i^kdotH_j^iH_k^j\n\nwhere H^beta_alpha is the homogeneous transform from frame alpha to frame beta, and hatx is the 3 times 3 skew symmetric matrix that satisfies hatx y = x times y for all y in mathbbR^3.\n\nHere, omega_j^ki is the angular part and v_j^ki is the linear part. Note that the linear part is not in general the same as the linear velocity of the origin of frame j.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.SpatialAcceleration",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.SpatialAcceleration",
    "category": "type",
    "text": "struct SpatialAcceleration{T}\n\nA spatial acceleration is the time derivative of a twist.\n\nSee Twist.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Twists,-spatial-accelerations-1",
    "page": "Spatial vector algebra",
    "title": "Twists, spatial accelerations",
    "category": "section",
    "text": "Twist\nSpatialAcceleration"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.Momentum",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.Momentum",
    "category": "type",
    "text": "struct Momentum{T}\n\nA Momentum is the product of a SpatialInertia and a Twist, i.e.\n\nh^i =\nleft(beginarrayc\nk^i\nl^i\nendarrayright) =\nI^i T^i j_k\n\nwhere I^i is the spatial inertia of a given body expressed in frame i, and T^i j_k is the twist of frame k (attached to the body) with respect to inertial frame j, expressed in frame i. k^i is the angular momentum and l^i is the linear momentum.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.Wrench",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.Wrench",
    "category": "type",
    "text": "struct Wrench{T}\n\nA wrench represents a system of forces.\n\nThe wrench w^i expressed in frame i is defined as\n\nw^i =\nleft(beginarrayc\ntau^i\nf^i\nendarrayright) =\nsum_jleft(beginarrayc\nr_j^itimes f_j^i\nf_j^i\nendarrayright)\n\nwhere the f_j^i are forces expressed in frame i, exerted at positions r_j^i. tau^i is the total torque and f^i is the total force.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Momenta,-wrenches-1",
    "page": "Spatial vector algebra",
    "title": "Momenta, wrenches",
    "category": "section",
    "text": "Momentum\nWrench"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.GeometricJacobian",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.GeometricJacobian",
    "category": "type",
    "text": "struct GeometricJacobian{A<:(AbstractArray{T,2} where T)}\n\nA geometric Jacobian (also known as basic, or spatial Jacobian) maps a vector of joint velocities to a twist.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Geometric-Jacobians-1",
    "page": "Spatial vector algebra",
    "title": "Geometric Jacobians",
    "category": "section",
    "text": "GeometricJacobian"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.MomentumMatrix",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.MomentumMatrix",
    "category": "type",
    "text": "struct MomentumMatrix{A<:(AbstractArray{T,2} where T)}\n\nA momentum matrix maps a joint velocity vector to momentum.\n\nThis is a slight generalization of the centroidal momentum matrix (Orin, Goswami, \"Centroidal momentum matrix of a humanoid robot: Structure and properties.\") in that the matrix (and hence the corresponding total momentum) need not be expressed in a centroidal frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Momentum-matrices-1",
    "page": "Spatial vector algebra",
    "title": "Momentum matrices",
    "category": "section",
    "text": "MomentumMatrix"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.@framecheck",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.@framecheck",
    "category": "macro",
    "text": "Check that CartesianFrame3D f1 is one of f2s.\n\nNote that if f2s is a CartesianFrame3D, then f1 and f2s are simply checked for equality.\n\nThrows an ArgumentError if f1 is not among f2s when bounds checks are enabled. @framecheck is a no-op when bounds checks are disabled.\n\n\n\n\n\n"
},

{
    "location": "spatial/#The-@framecheck-macro-1",
    "page": "Spatial vector algebra",
    "title": "The @framecheck macro",
    "category": "section",
    "text": "@framecheck"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.center_of_mass-Tuple{SpatialInertia}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.center_of_mass",
    "category": "method",
    "text": "center_of_mass(inertia)\n\n\nReturn the center of mass of the SpatialInertia as a Point3D.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.kinetic_energy-Tuple{SpatialInertia,Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.kinetic_energy",
    "category": "method",
    "text": "kinetic_energy(inertia, twist)\n\n\nCompute the kinetic energy of a body with spatial inertia I, which has twist T with respect to an inertial frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.log_with_time_derivative-Tuple{Transform3D,Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.log_with_time_derivative",
    "category": "method",
    "text": "log_with_time_derivative(t, twist)\n\n\nCompute exponential coordinates as well as their time derivatives in one shot. This mainly exists because ForwardDiff won\'t work at the singularity of log. It is also ~50% faster than ForwardDiff in this case.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.newton_euler-Tuple{SpatialInertia,SpatialAcceleration,Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.newton_euler",
    "category": "method",
    "text": "newton_euler(inertia, spatial_accel, twist)\n\n\nApply the Newton-Euler equations to find the external wrench required to make a body with spatial inertia I, which has twist T with respect to an inertial frame, achieve spatial acceleration dotT.\n\nThis wrench is also equal to the rate of change of momentum of the body.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.point_acceleration-Tuple{Twist,SpatialAcceleration,Point3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.point_acceleration",
    "category": "method",
    "text": "point_acceleration(twist, accel, point)\n\n\nGiven the twist dotT_j^ki of frame j with respect to frame i, expressed in frame k and its time derivative (a spatial acceleration), as well as the location of a point fixed in frame j, also expressed in frame k, compute the acceleration of the point relative to frame i.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.point_velocity-Tuple{Twist,Point3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.point_velocity",
    "category": "method",
    "text": "point_velocity(twist, point)\n\n\nGiven the twist T_j^ki of frame j with respect to frame i, expressed in frame k, and the location of a point fixed in frame j, also expressed in frame k, compute the velocity of the point relative to frame i.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{FreeVector3D,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(x, t)\n\n\nReturn x transformed to CartesianFrame3D t.from.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{GeometricJacobian,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(jac, tf)\n\n\nTransform the GeometricJacobian to a different frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{Momentum,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(f, tf)\n\n\nTransform the Momentum to a different frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{Point3D,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(x, t)\n\n\nReturn x transformed to CartesianFrame3D t.from.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{SpatialAcceleration,Transform3D,Twist,Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(accel, old_to_new, twist_of_current_wrt_new, twist_of_body_wrt_base)\n\n\nTransform the SpatialAcceleration to a different frame.\n\nThe transformation rule is obtained by differentiating the transformation rule for twists.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{SpatialInertia,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(inertia, t)\n\n\nTransform the SpatialInertia to a different frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{Twist,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(twist, tf)\n\n\nTransform the Twist to a different frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.transform-Tuple{Wrench,Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.transform",
    "category": "method",
    "text": "transform(f, tf)\n\n\nTransform the Wrench to a different frame.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Base.exp-Tuple{Twist}",
    "page": "Spatial vector algebra",
    "title": "Base.exp",
    "category": "method",
    "text": "exp(twist)\n\n\nConvert exponential coordinates to a homogeneous transform.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Base.log-Tuple{Transform3D}",
    "page": "Spatial vector algebra",
    "title": "Base.log",
    "category": "method",
    "text": "log(t)\n\n\nExpress a homogeneous transform in exponential coordinates centered around the identity.\n\n\n\n\n\n"
},

{
    "location": "spatial/#LinearAlgebra.dot-Tuple{Wrench,Twist}",
    "page": "Spatial vector algebra",
    "title": "LinearAlgebra.dot",
    "category": "method",
    "text": "dot(w, t)\n\n\nCompute the mechanical power associated with a pairing of a wrench and a twist.\n\n\n\n\n\n"
},

{
    "location": "spatial/#RigidBodyDynamics.Spatial.colwise",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Spatial.colwise",
    "category": "function",
    "text": "Equivalent to one of\n\nmapslices(x -> f(a, x), B, dims=1)\nmapslices(x -> f(x, b), A, dims=1)\n\nbut optimized for statically-sized matrices.\n\n\n\n\n\n"
},

{
    "location": "spatial/#Functions-1",
    "page": "Spatial vector algebra",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics.Spatial]\nOrder   = [:function]"
},

{
    "location": "joints/#",
    "page": "Joints",
    "title": "Joints",
    "category": "page",
    "text": ""
},

{
    "location": "joints/#Joints-1",
    "page": "Joints",
    "title": "Joints",
    "category": "section",
    "text": ""
},

{
    "location": "joints/#Index-1",
    "page": "Joints",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"joints.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "joints/#RigidBodyDynamics.Joint",
    "page": "Joints",
    "title": "RigidBodyDynamics.Joint",
    "category": "type",
    "text": "struct Joint{T, JT<:JointType{T}}\n\nA joint represents a kinematic restriction of the relative twist between two rigid bodies to a linear subspace of dimension k.\n\nA joint has a direction. The rigid body before the joint is called the joint\'s predecessor, and the rigid body after the joint is its successor.\n\nThe state related to the joint is parameterized by two sets of variables, namely\n\na vector q in mathcalQ, parameterizing the relative homogeneous transform.\na vector v in mathbbR^k, parameterizing the relative twist.\n\nThe twist of the successor with respect to the predecessor is a linear function of v.\n\nFor some joint types (notably those using a redundant representation of relative orientation, such as a unit quaternion), dotq, the time derivative of q, may not be the same as v. However, an invertible linear transformation exists between dotq and v.\n\nSee also:\n\nDefinition 2.9 in Duindam, \"Port-Based Modeling and Control for Efficient Bipedal Walking Robots\", 2006.\nSection 4.4 of Featherstone, \"Rigid Body Dynamics Algorithms\", 2008.\n\n\n\n\n\n"
},

{
    "location": "joints/#The-Joint-type-1",
    "page": "Joints",
    "title": "The Joint type",
    "category": "section",
    "text": "Joint"
},

{
    "location": "joints/#RigidBodyDynamics.bias_acceleration-Tuple{Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "method",
    "text": "bias_acceleration(joint, q, v)\n\n\nReturn the acceleration of the joint\'s successor with respect to its predecessor in configuration q and at velocity v, when the joint acceleration dotv is zero.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.configuration_derivative_to_velocity!-Tuple{AbstractArray{T,1} where T,Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.configuration_derivative_to_velocity!",
    "category": "method",
    "text": "configuration_derivative_to_velocity!(v, joint, q, q̇)\n\n\nCompute joint velocity vector v given the joint configuration vector q and its time derivative dotq (in place).\n\nNote that this mapping is linear.\n\nSee also velocity_to_configuration_derivative!, the inverse mapping.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.configuration_derivative_to_velocity_adjoint!-Tuple{Any,Joint,AbstractArray{T,1} where T,Any}",
    "page": "Joints",
    "title": "RigidBodyDynamics.configuration_derivative_to_velocity_adjoint!",
    "category": "method",
    "text": "configuration_derivative_to_velocity_adjoint!(fq, joint, q, fv)\n\n\nGiven  a linear function\n\nf(v) = langle f_v v rangle\n\nwhere v is the joint velocity vector, return a vector f_q such that\n\nlangle f_v v rangle = langle f_q dotq(v) rangle\n\nNote: since v is a linear function of dotq (see configuration_derivative_to_velocity!), we can write v = J_dotq rightarrow v dotq, so\n\nlangle f_v v rangle = langle f_v J_dotq rightarrow v dotq rangle = langle J_dotq rightarrow v^* f_v dotq rangle\n\nso f_q = J_dotq rightarrow v^* f_v.\n\nTo compute J_dotq rightarrow v see configuration_derivative_to_velocity_jacobian.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.constraint_wrench_subspace-Tuple{Joint,Transform3D}",
    "page": "Joints",
    "title": "RigidBodyDynamics.constraint_wrench_subspace",
    "category": "method",
    "text": "constraint_wrench_subspace(joint, joint_transform)\n\n\nReturn a basis for the constraint wrench subspace of the joint, where joint_transform is the transform from the frame after the joint to the frame before the joint.\n\nThe constraint wrench subspace is a 6 times (6 - k) matrix, where k is the dimension of the velocity vector v, that maps a vector of Lagrange multipliers lambda to the constraint wrench exerted across the joint onto its successor.\n\nThe constraint wrench subspace is orthogonal to the motion subspace.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.effort_bounds-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.effort_bounds",
    "category": "method",
    "text": "effort_bounds(joint)\n\n\nReturn a Vector{Bounds{T}} giving the upper and lower bounds of the effort for joint\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.global_coordinates!-Tuple{AbstractArray{T,1} where T,Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.global_coordinates!",
    "category": "method",
    "text": "global_coordinates!(q, joint, q0, ϕ)\n\n\nCompute the global parameterization of the joint\'s configuration, q, given a \'base\' orientation q_0 and a vector of local coordinates ϕ centered around q_0.\n\nSee also local_coordinates!.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.has_fixed_subspaces-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.has_fixed_subspaces",
    "category": "method",
    "text": "has_fixed_subspaces(joint)\n\n\nWhether the joint\'s motion subspace and constraint wrench subspace depend on q.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.isfloating-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.isfloating",
    "category": "method",
    "text": "isfloating(joint)\n\n\nWhether the joint is a floating joint, i.e., whether it imposes no constraints on the relative motions of its successor and predecessor bodies.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.joint_transform-Tuple{Joint,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_transform",
    "category": "method",
    "text": "joint_transform(joint, q)\n\n\nReturn a Transform3D representing the homogeneous transform from the frame after the joint to the frame before the joint for joint configuration vector q.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.local_coordinates!-Tuple{AbstractArray{T,1} where T,AbstractArray{T,1} where T,Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.local_coordinates!",
    "category": "method",
    "text": "local_coordinates!(ϕ, ϕ̇, joint, q0, q, v)\n\n\nCompute a vector of local coordinates phi around configuration q_0 corresponding to configuration q (in place). Also compute the time derivative dotphi of phi given the joint velocity vector v.\n\nThe local coordinate vector phi must be zero if and only if q = q_0.\n\nFor revolute or prismatic joint types, the local coordinates can just be phi = q - q_0, but for joint types with configuration vectors that are restricted to a manifold (e.g. when unit quaternions are used to represent orientation), elementwise subtraction may not make sense. For such joints, exponential coordinates could be used as the local coordinate vector phi.\n\nSee also global_coordinates!.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.motion_subspace-Tuple{Joint,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.motion_subspace",
    "category": "method",
    "text": "motion_subspace(joint, q)\n\n\nReturn a basis for the motion subspace of the joint in configuration q.\n\nThe motion subspace basis is a 6 times  k matrix, where k is the dimension of the velocity vector v, that maps v to the twist of the joint\'s successor with respect to its predecessor. The returned motion subspace is expressed in the frame after the joint, which is attached to the joint\'s successor.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.normalize_configuration!-Tuple{AbstractArray{T,1} where T,Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.normalize_configuration!",
    "category": "method",
    "text": "normalize_configuration!(q, joint)\n\n\nRenormalize the configuration vector q associated with joint so that it lies on the joint\'s configuration manifold.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.num_constraints-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_constraints",
    "category": "method",
    "text": "num_constraints(joint)\n\n\nReturn the number of constraints imposed on the relative twist between the joint\'s predecessor and successor\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.num_positions-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_positions",
    "category": "method",
    "text": "num_positions(joint)\n\n\nReturn the length of the configuration vector of joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.num_velocities-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "method",
    "text": "num_velocities(joint)\n\n\nReturn the length of the velocity vector of joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.position_bounds-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.position_bounds",
    "category": "method",
    "text": "position_bounds(joint)\n\n\nReturn a Vector{Bounds{T}} giving the upper and lower bounds of the configuration for joint\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.principal_value!-Tuple{AbstractArray{T,1} where T,Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.principal_value!",
    "category": "method",
    "text": "principal_value!(q, joint)\n\n\nApplies the principalvalue functions from [Rotations.jl](https://github.com/FugroRoames/Rotations.jl/blob/d080990517f89b56c37962ad53a7fd24bd94b9f7/src/principalvalue.jl) to joint angles. This currently only applies to SPQuatFloating joints.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.rand_configuration!-Tuple{AbstractArray{T,1} where T,Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.rand_configuration!",
    "category": "method",
    "text": "rand_configuration!(q, joint)\n\n\nSet q to a random configuration. The distribution used depends on the joint type.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.velocity_bounds-Tuple{Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.velocity_bounds",
    "category": "method",
    "text": "velocity_bounds(joint)\n\n\nReturn a Vector{Bounds{T}} giving the upper and lower bounds of the velocity for joint\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.velocity_to_configuration_derivative!-Tuple{AbstractArray{T,1} where T,Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.velocity_to_configuration_derivative!",
    "category": "method",
    "text": "velocity_to_configuration_derivative!(q̇, joint, q, v)\n\n\nCompute the time derivative dotq of the joint configuration vector q given q and the joint velocity vector v (in place).\n\nNote that this mapping is linear.\n\nSee also configuration_derivative_to_velocity!, the inverse mapping.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.zero_configuration!-Tuple{AbstractArray{T,1} where T,Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.zero_configuration!",
    "category": "method",
    "text": "zero_configuration!(q, joint)\n\n\nSet q to the \'zero\' configuration, corresponding to an identity joint transform.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.configuration_derivative_to_velocity_jacobian-Tuple{Joint,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.configuration_derivative_to_velocity_jacobian",
    "category": "method",
    "text": "configuration_derivative_to_velocity_jacobian(joint, q)\n\n\nCompute the jacobian J_dotq rightarrow v which maps joint configuration derivative to velocity for the given joint:\n\nv = J_dotq rightarrow v dotq\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.joint_spatial_acceleration-Tuple{Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_spatial_acceleration",
    "category": "method",
    "text": "joint_spatial_acceleration(joint, q, v, vd)\n\n\nReturn the spatial acceleration of joint\'s  successor with respect to its predecessor, expressed in the frame after the joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.joint_torque!-Tuple{AbstractArray{T,1} where T,Joint,AbstractArray{T,1} where T,Wrench}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_torque!",
    "category": "method",
    "text": "joint_torque!(τ, joint, q, joint_wrench)\n\n\nGiven the wrench exerted across the joint on the joint\'s successor, compute the vector of joint torques tau (in place), in configuration q.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.joint_twist-Tuple{Joint,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_twist",
    "category": "method",
    "text": "joint_twist(joint, q, v)\n\n\nReturn the twist of joint\'s  successor with respect to its predecessor, expressed in the frame after the joint.\n\nNote that this is the same as Twist(motion_subspace(joint, q), v).\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.velocity_to_configuration_derivative_jacobian-Tuple{Joint,AbstractArray{T,1} where T}",
    "page": "Joints",
    "title": "RigidBodyDynamics.velocity_to_configuration_derivative_jacobian",
    "category": "method",
    "text": "velocity_to_configuration_derivative_jacobian(joint, q)\n\n\nCompute the jacobian J_v rightarrow dotq which maps joint velocity to configuration derivative for the given joint:\n\ndotq = J_v rightarrow dotq v\n\n\n\n\n\n"
},

{
    "location": "joints/#Functions-1",
    "page": "Joints",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"joint.jl\"]"
},

{
    "location": "joints/#RigidBodyDynamics.JointType",
    "page": "Joints",
    "title": "RigidBodyDynamics.JointType",
    "category": "type",
    "text": "abstract type JointType\n\nThe abstract supertype of all concrete joint types.\n\n\n\n\n\n"
},

{
    "location": "joints/#JointTypes-1",
    "page": "Joints",
    "title": "JointTypes",
    "category": "section",
    "text": "JointType"
},

{
    "location": "joints/#RigidBodyDynamics.Fixed",
    "page": "Joints",
    "title": "RigidBodyDynamics.Fixed",
    "category": "type",
    "text": "struct Fixed{T} <: JointType{T}\n\nThe Fixed joint type is a degenerate joint type, in the sense that it allows no motion between its predecessor and successor rigid bodies.\n\n\n\n\n\n"
},

{
    "location": "joints/#Fixed-1",
    "page": "Joints",
    "title": "Fixed",
    "category": "section",
    "text": "Fixed"
},

{
    "location": "joints/#RigidBodyDynamics.Revolute",
    "page": "Joints",
    "title": "RigidBodyDynamics.Revolute",
    "category": "type",
    "text": "struct Revolute{T} <: JointType{T}\n\nA Revolute joint type allows rotation about a fixed axis.\n\nThe configuration vector for the Revolute joint type simply consists of the angle of rotation about the specified axis. The velocity vector consists of the angular rate, and is thus the time derivative of the configuration vector.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.Revolute-Tuple{Any}",
    "page": "Joints",
    "title": "RigidBodyDynamics.Revolute",
    "category": "method",
    "text": "Revolute(axis)\n\n\nConstruct a new Revolute joint type, allowing rotation about axis (expressed in the frame before the joint).\n\n\n\n\n\n"
},

{
    "location": "joints/#Revolute-1",
    "page": "Joints",
    "title": "Revolute",
    "category": "section",
    "text": "Revolute\nRevolute(axis)"
},

{
    "location": "joints/#RigidBodyDynamics.Prismatic",
    "page": "Joints",
    "title": "RigidBodyDynamics.Prismatic",
    "category": "type",
    "text": "struct Prismatic{T} <: JointType{T}\n\nA Prismatic joint type allows translation along a fixed axis.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.Prismatic-Tuple{Any}",
    "page": "Joints",
    "title": "RigidBodyDynamics.Prismatic",
    "category": "method",
    "text": "Construct a new Prismatic joint type, allowing translation along axis (expressed in the frame before the joint).\n\n\n\n\n\n"
},

{
    "location": "joints/#Prismatic-1",
    "page": "Joints",
    "title": "Prismatic",
    "category": "section",
    "text": "Prismatic\nPrismatic(axis)"
},

{
    "location": "joints/#RigidBodyDynamics.Planar",
    "page": "Joints",
    "title": "RigidBodyDynamics.Planar",
    "category": "type",
    "text": "struct Planar{T} <: JointType{T}\n\nThe Planar joint type allows translation along two orthogonal vectors, referred to as x and y, as well as rotation about an axis z = x times y.\n\nThe components of the 3-dimensional configuration vector q associated with a Planar joint are the x- and y-coordinates of the translation, and the angle of rotation theta about z, in that order.\n\nThe components of the 3-dimension velocity vector v associated with a Planar joint are the x- and y-coordinates of the linear part of the joint twist, expressed in the frame after the joint, followed by the z-component of the angular part of this joint twist.\n\nwarning: Warning\nFor the Planar joint type, v neq dotq! Although the angular parts of v and dotq are the same, their linear parts differ. The linear part of v is the linear part of dotq, rotated to the frame after the joint. This parameterization was chosen to allow the translational component of the joint transform to be independent of the rotation angle theta (i.e., the rotation is applied after the translation), while still retaining a constant motion subspace expressed in the frame after the joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.Planar-Union{Tuple{T}, Tuple{AbstractArray{T,1} where T,AbstractArray{T,1} where T}} where T",
    "page": "Joints",
    "title": "RigidBodyDynamics.Planar",
    "category": "method",
    "text": "Planar(x_axis, y_axis)\n\n\nConstruct a new Planar joint type with the xy-plane in which translation is allowed defined by 3-vectors x and y expressed in the frame before the joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#Planar-1",
    "page": "Joints",
    "title": "Planar",
    "category": "section",
    "text": "Planar\nPlanar{T}(x_axis::AbstractVector, y_axis::AbstractVector) where {T}"
},

{
    "location": "joints/#RigidBodyDynamics.QuaternionSpherical",
    "page": "Joints",
    "title": "RigidBodyDynamics.QuaternionSpherical",
    "category": "type",
    "text": "struct QuaternionSpherical{T} <: JointType{T}\n\nThe QuaternionSpherical joint type allows rotation in any direction. It is an implementation of a ball-and-socket joint.\n\nThe 4-dimensional configuration vector q associated with a QuaternionSpherical joint is the unit quaternion that describes the orientation of the frame after the joint with respect to the frame before the joint. In other words, it is the quaternion that can be used to rotate vectors from the frame after the joint to the frame before the joint.\n\nThe 3-dimensional velocity vector v associated with a QuaternionSpherical joint is the angular velocity of the frame after the joint with respect to the frame before the joint, expressed in the frame after the joint (body frame).\n\n\n\n\n\n"
},

{
    "location": "joints/#QuaternionSpherical-1",
    "page": "Joints",
    "title": "QuaternionSpherical",
    "category": "section",
    "text": "QuaternionSpherical"
},

{
    "location": "joints/#RigidBodyDynamics.QuaternionFloating",
    "page": "Joints",
    "title": "RigidBodyDynamics.QuaternionFloating",
    "category": "type",
    "text": "struct QuaternionFloating{T} <: JointType{T}\n\nA floating joint type that uses a unit quaternion representation for orientation.\n\nFloating joints are 6-degree-of-freedom joints that are in a sense degenerate, as they impose no constraints on the relative motion between two bodies.\n\nThe full, 7-dimensional configuration vector of a QuaternionFloating joint type consists of a unit quaternion representing the orientation that rotates vectors from the frame \'directly after\' the joint to the frame \'directly before\' it, and a 3D position vector representing the origin of the frame after the joint in the frame before the joint.\n\nThe 6-dimensional velocity vector of a QuaternionFloating joint is the twist of the frame after the joint with respect to the frame before it, expressed in the frame after the joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#QuaternionFloating-1",
    "page": "Joints",
    "title": "QuaternionFloating",
    "category": "section",
    "text": "QuaternionFloating"
},

{
    "location": "joints/#RigidBodyDynamics.SPQuatFloating",
    "page": "Joints",
    "title": "RigidBodyDynamics.SPQuatFloating",
    "category": "type",
    "text": "struct SPQuatFloating{T} <: JointType{T}\n\nA floating joint type that uses a SPQuat representation for orientation.\n\nFloating joints are 6-degree-of-freedom joints that are in a sense degenerate, as they impose no constraints on the relative motion between two bodies.\n\nThe 6-dimensional configuration vector of a SPQuatFloating joint type consists of a SPQuat representing the orientation that rotates vectors from the frame \'directly after\' the joint to the frame \'directly before\' it, and a 3D position vector representing the origin of the frame after the joint in the frame before the joint.\n\nThe 6-dimensional velocity vector of a SPQuatFloating joint is the twist of the frame after the joint with respect to the frame before it, expressed in the frame after the joint.\n\n\n\n\n\n"
},

{
    "location": "joints/#SPQuatFloating-1",
    "page": "Joints",
    "title": "SPQuatFloating",
    "category": "section",
    "text": "SPQuatFloating"
},

{
    "location": "joints/#RigidBodyDynamics.SinCosRevolute",
    "page": "Joints",
    "title": "RigidBodyDynamics.SinCosRevolute",
    "category": "type",
    "text": "struct SinCosRevolute{T} <: JointType{T}\n\nA SinCosRevolute joint type allows rotation about a fixed axis.\n\nIn contrast to the Revolute joint type, the configuration vector for the SinCosRevolute joint type consists of the sine and cosine of the angle of rotation about the specified axis (in that order). The velocity vector for the SinCosRevolute joint type is the same as for the Revolute joint type, i.e., the time derivative of the angle about the axis.\n\n\n\n\n\n"
},

{
    "location": "joints/#RigidBodyDynamics.SinCosRevolute-Tuple{Any}",
    "page": "Joints",
    "title": "RigidBodyDynamics.SinCosRevolute",
    "category": "method",
    "text": "Construct a new SinCosRevolute joint type, allowing rotation about axis (expressed in the frame before the joint).\n\n\n\n\n\n"
},

{
    "location": "joints/#SinCosRevolute-1",
    "page": "Joints",
    "title": "SinCosRevolute",
    "category": "section",
    "text": "SinCosRevolute\nSinCosRevolute(axis)"
},

{
    "location": "rigidbody/#",
    "page": "Rigid bodies",
    "title": "Rigid bodies",
    "category": "page",
    "text": ""
},

{
    "location": "rigidbody/#Rigid-bodies-1",
    "page": "Rigid bodies",
    "title": "Rigid bodies",
    "category": "section",
    "text": ""
},

{
    "location": "rigidbody/#Index-1",
    "page": "Rigid bodies",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"rigidbody.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "rigidbody/#RigidBodyDynamics.RigidBody",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.RigidBody",
    "category": "type",
    "text": "mutable struct RigidBody{T}\n\nA non-deformable body.\n\nA RigidBody has inertia (represented as a SpatialInertia), unless it represents a root (world) body. A RigidBody additionally stores a list of definitions of coordinate systems that are rigidly attached to it.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#The-RigidBody-type-1",
    "page": "Rigid bodies",
    "title": "The RigidBody type",
    "category": "section",
    "text": "RigidBody"
},

{
    "location": "rigidbody/#RigidBodyDynamics.add_contact_point!-Union{Tuple{T}, Tuple{RigidBody{T},ContactPoint{T,SoftContactModel{HuntCrossleyModel{T},ViscoelasticCoulombModel{T}}}}} where T",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.add_contact_point!",
    "category": "method",
    "text": "add_contact_point!(body, point)\n\n\nAdd a new contact point to the rigid body\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.add_frame!-Tuple{RigidBody,Transform3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.add_frame!",
    "category": "method",
    "text": "add_frame!(body, transform)\n\n\nAdd a new frame definition to body, represented by a homogeneous transform from the CartesianFrame3D to be added to any other frame that is already attached to body.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.contact_points-Tuple{RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.contact_points",
    "category": "method",
    "text": "contact_points(body)\n\n\nReturn the contact points attached to the body as an ordered collection.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.default_frame-Tuple{RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.default_frame",
    "category": "method",
    "text": "default_frame(body)\n\n\nThe CartesianFrame3D with respect to which all other frames attached to body are defined.\n\nSee frame_definitions(body), frame_definition(body, frame).\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.fixed_transform-Tuple{RigidBody,CartesianFrame3D,CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.fixed_transform",
    "category": "method",
    "text": "fixed_transform(body, from, to)\n\n\nReturn the transform from CartesianFrame3D from to to, both of which are rigidly attached to body.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.has_defined_inertia-Tuple{RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.has_defined_inertia",
    "category": "method",
    "text": "has_defined_inertia(b)\n\n\nWhether the body has a defined inertia.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.spatial_inertia!-Tuple{RigidBody,SpatialInertia}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.spatial_inertia!",
    "category": "method",
    "text": "spatial_inertia!(body, inertia)\n\n\nSet the spatial inertia of the body.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.spatial_inertia-Union{Tuple{RigidBody{T}}, Tuple{T}} where T",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.spatial_inertia",
    "category": "method",
    "text": "Return the spatial inertia of the body. If the inertia is undefined, calling this method will result in an error.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.change_default_frame!-Tuple{RigidBody,CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.change_default_frame!",
    "category": "method",
    "text": "change_default_frame!(body, new_default_frame)\n\n\nChange the default frame of body to frame (which should already be among body\'s frame definitions).\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.frame_definition-Tuple{RigidBody,CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.frame_definition",
    "category": "method",
    "text": "frame_definition(body, frame)\n\n\nReturn the Transform3D defining frame (attached to body) with respect to default_frame(body).\n\nThrows an error if frame is not attached to body.\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.frame_definitions-Tuple{RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.frame_definitions",
    "category": "method",
    "text": "frame_definitions(body)\n\nReturn the list of homogeneous transforms (Transform3Ds) that define the coordinate systems attached to body with respect to a single common frame (default_frame(body)).\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#RigidBodyDynamics.is_fixed_to_body-Tuple{RigidBody,CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.is_fixed_to_body",
    "category": "method",
    "text": "is_fixed_to_body(body, frame)\n\n\nWhether frame is attached to body (i.e. whether it is among frame_definitions(body)).\n\n\n\n\n\n"
},

{
    "location": "rigidbody/#Functions-1",
    "page": "Rigid bodies",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"rigid_body.jl\"]"
},

{
    "location": "mechanism/#",
    "page": "Mechanism",
    "title": "Mechanism",
    "category": "page",
    "text": ""
},

{
    "location": "mechanism/#Mechanisms-1",
    "page": "Mechanism",
    "title": "Mechanisms",
    "category": "section",
    "text": ""
},

{
    "location": "mechanism/#Index-1",
    "page": "Mechanism",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"mechanism.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "mechanism/#RigidBodyDynamics.Mechanism",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.Mechanism",
    "category": "type",
    "text": "mutable struct Mechanism{T}\n\nA Mechanism represents an interconnection of rigid bodies and joints. Mechanisms store the joint layout and inertia parameters, but no state-dependent information.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#The-Mechanism-type-1",
    "page": "Mechanism",
    "title": "The Mechanism type",
    "category": "section",
    "text": "Mechanism"
},

{
    "location": "mechanism/#RigidBodyDynamics.Mechanism-Tuple{Any}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.Mechanism",
    "category": "method",
    "text": "Create a new Mechanism containing only a root body, to which other bodies can be attached with joints.\n\nThe gravity keyword argument can be used to set the gravitational acceleration (a 3-vector expressed in the Mechanism\'s root frame). Default: [0.0, 0.0, -9.81].\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.attach!-Union{Tuple{T}, Tuple{Mechanism{T},RigidBody{T},Mechanism{T}}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.attach!",
    "category": "method",
    "text": "attach!(mechanism, parentbody, childmechanism; child_root_pose, bodymap, jointmap)\n\n\nAttach a copy of childmechanism to mechanism.\n\nEssentially replaces the root body of a copy of childmechanism with parentbody (which belongs to mechanism).\n\nNote: gravitational acceleration for childmechanism is ignored.\n\nKeyword arguments:\n\nchild_root_pose: pose of the default frame of the root body of childmechanism relative to that of parentbody. Default: the identity transformation.\nbodymap::AbstractDict{RigidBody{T}, RigidBody{T}}: will be populated with the mapping from input mechanism\'s bodies to output mechanism\'s bodies\njointmap::AbstractDict{<:Joint{T}, <:Joint{T}}: will be populated with the mapping from input mechanism\'s joints to output mechanism\'s joints\n\nwhere T is the element type of mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.attach!-Union{Tuple{T}, Tuple{Mechanism{T},RigidBody{T},RigidBody{T},Joint{T,JT} where JT<:JointType{T}}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.attach!",
    "category": "method",
    "text": "attach!(mechanism, predecessor, successor, joint; joint_pose, successor_pose)\n\n\nAttach successor to predecessor using joint.\n\nSee Joint for definitions of the terms successor and predecessor.\n\nThe Transform3Ds joint_pose and successor_pose define where joint is attached to each body. joint_pose should define frame_before(joint) with respect to any frame fixed to predecessor, and likewise successor_pose should define any frame fixed to successor with respect to frame_after(joint).\n\npredecessor is required to already be among the bodies of the Mechanism.\n\nIf successor is not yet a part of the Mechanism, it will be added to the Mechanism. Otherwise, the joint will be treated as a non-tree edge in the Mechanism, effectively creating a loop constraint that will be enforced using Lagrange multipliers (as opposed to using recursive algorithms).\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.maximal_coordinates-Union{Tuple{Mechanism{T}}, Tuple{T}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.maximal_coordinates",
    "category": "method",
    "text": "Return a dynamically equivalent Mechanism, but with a flat tree structure: all bodies are attached to the root body via a floating joint, and the tree joints of the input Mechanism are transformed into non-tree joints (with joint constraints enforced using Lagrange multipliers in dynamics!).\n\nKeyword arguments:\n\nfloating_joint_type::Type{<:JointType{T}}: which floating joint type to use for the joints between each body and the root. Default: QuaternionFloating{T}\nbodymap::AbstractDict{RigidBody{T}, RigidBody{T}}: will be populated with the mapping from input mechanism\'s bodies to output mechanism\'s bodies\njointmap::AbstractDict{<:Joint{T}, <:Joint{T}}: will be populated with the mapping from input mechanism\'s joints to output mechanism\'s joints\n\nwhere T is the element type of mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.rand_chain_mechanism-Union{Tuple{T}, Tuple{Type{T},Vararg{Type{#s148} where #s148<:JointType{T},N} where N}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_chain_mechanism",
    "category": "method",
    "text": "Create a random chain Mechanism with the given joint types.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.rand_floating_tree_mechanism-Union{Tuple{T}, Tuple{Type{T},Vararg{Type{#s148} where #s148<:JointType{T},N} where N}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_floating_tree_mechanism",
    "category": "method",
    "text": "rand_floating_tree_mechanism(?, nonfloatingjointtypes)\n\n\nCreate a random tree Mechanism, with a quaternion floating joint as the first joint (between the root body and the first non-root body).\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.rand_tree_mechanism-Union{Tuple{T}, Tuple{Type{T},Function,Vararg{Type{#s144} where #s144<:JointType{T},N} where N}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_tree_mechanism",
    "category": "method",
    "text": "rand_tree_mechanism(?, parentselector, jointtypes)\n\n\nCreate a random tree Mechanism with the given joint types. Each new body is attached to a parent selected using the parentselector function.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.rand_tree_mechanism-Union{Tuple{T}, Tuple{Type{T},Vararg{Type{#s148} where #s148<:JointType{T},N} where N}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_tree_mechanism",
    "category": "method",
    "text": "Create a random tree Mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.remove_fixed_tree_joints!-Union{Tuple{Mechanism{T}}, Tuple{T}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.remove_fixed_tree_joints!",
    "category": "method",
    "text": "Remove any fixed joints present as tree edges in mechanism by merging the rigid bodies that these fixed joints join together into bodies with equivalent inertial properties.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.remove_joint!-Union{Tuple{M}, Tuple{Mechanism{M},Joint{M,JT} where JT<:JointType{M}}} where M",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.remove_joint!",
    "category": "method",
    "text": "remove_joint!(mechanism, joint; flipped_joint_map, spanning_tree_next_edge)\n\n\nRemove a joint from the mechanism. Rebuilds the spanning tree if the joint is part of the current spanning tree.\n\nOptionally, the flipped_joint_map keyword argument can be used to pass in an associative container that will be populated with a mapping from original joints to flipped joints, if removing joint requires rebuilding the spanning tree of mechanism and the polarity of some joints needed to be changed in the process.\n\nAlso optionally, spanning_tree_next_edge can be used to select which joints should become part of the new spanning tree, if rebuilding the spanning tree is required.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.submechanism-Union{Tuple{T}, Tuple{Mechanism{T},RigidBody{T}}} where T",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.submechanism",
    "category": "method",
    "text": "submechanism(mechanism, submechanismroot; bodymap, jointmap)\n\n\nCreate a new Mechanism from the subtree of mechanism rooted at submechanismroot.\n\nAny non-tree joint in mechanism will appear in the returned Mechanism if and only if both its successor and its predecessor are part of the subtree.\n\nKeyword arguments:\n\nbodymap::AbstractDict{RigidBody{T}, RigidBody{T}}: will be populated with the mapping from input mechanism\'s bodies to output mechanism\'s bodies\njointmap::AbstractDict{<:Joint{T}, <:Joint{T}}: will be populated with the mapping from input mechanism\'s joints to output mechanism\'s joints\n\nwhere T is the element type of mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.rebuild_spanning_tree!-Union{Tuple{Mechanism{M}}, Tuple{M}, Tuple{Mechanism{M},Union{Nothing, AbstractDict}}} where M",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rebuild_spanning_tree!",
    "category": "method",
    "text": "rebuild_spanning_tree!(mechanism)\nrebuild_spanning_tree!(mechanism, flipped_joint_map; next_edge)\n\n\nReconstruct the mechanism\'s spanning tree.\n\nOptionally, the flipped_joint_map keyword argument can be used to pass in an associative container that will be populated with a mapping from original joints to flipped joints, if the rebuilding process required the polarity of some joints to be flipped.\n\nAlso optionally, next_edge can be used to select which joints should become part of the new spanning tree.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#mechanism_create-1",
    "page": "Mechanism",
    "title": "Creating and modifying Mechanisms",
    "category": "section",
    "text": "See also URDF parsing and writing for URDF file format support.Mechanism(root_body; gravity)Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_modification.jl\"]"
},

{
    "location": "mechanism/#RigidBodyDynamics.bodies-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.bodies",
    "category": "method",
    "text": "bodies(mechanism)\n\n\nReturn the RigidBodys that are part of the Mechanism as an iterable collection.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.body_fixed_frame_to_body-Tuple{Mechanism,CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.body_fixed_frame_to_body",
    "category": "method",
    "text": "body_fixed_frame_to_body(mechanism, frame)\n\n\nReturn the RigidBody to which frame is attached.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.findbody-Tuple{Mechanism,BodyID}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.findbody",
    "category": "method",
    "text": "findbody(mechanism, id)\n\n\nReturn the RigidBody with the given BodyID.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.findbody-Tuple{Mechanism,String}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.findbody",
    "category": "method",
    "text": "findbody(mechanism, name)\n\n\nReturn the RigidBody with the given name. Errors if there is no body with the given name, or if there\'s more than one.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.findjoint-Tuple{Mechanism,JointID}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.findjoint",
    "category": "method",
    "text": "findjoint(mechanism, id)\n\n\nReturn the Joint with the given JointID.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.findjoint-Tuple{Mechanism,String}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.findjoint",
    "category": "method",
    "text": "findjoint(mechanism, name)\n\n\nReturn the Joint with the given name. Errors if there is no joint with the given name, or if there\'s more than one.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.fixed_transform-Tuple{Mechanism,CartesianFrame3D,CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.fixed_transform",
    "category": "method",
    "text": "fixed_transform(mechanism, from, to)\n\n\nReturn the transform from CartesianFrame3D from to to, both of which are rigidly attached to the same RigidBody.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.in_joints-Tuple{RigidBody,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.in_joints",
    "category": "method",
    "text": "in_joints(body, mechanism)\n\n\nReturn the joints that have body as their successor.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.joint_to_parent-Tuple{RigidBody,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joint_to_parent",
    "category": "method",
    "text": "joint_to_parent(body, mechanism)\n\n\nReturn the joint that is part of the mechanism\'s kinematic tree and has body as its successor.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.joints-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joints",
    "category": "method",
    "text": "joints(mechanism)\n\n\nReturn the Joints that are part of the Mechanism as an iterable collection.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.joints_to_children-Tuple{RigidBody,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joints_to_children",
    "category": "method",
    "text": "joints_to_children(body, mechanism)\n\n\nReturn the joints that are part of the mechanism\'s kinematic tree and have body as their predecessor.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.non_tree_joints-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.non_tree_joints",
    "category": "method",
    "text": "non_tree_joints(mechanism)\n\n\nReturn the Joints that are not part of the Mechanism\'s spanning tree as an iterable collection.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.num_additional_states-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_additional_states",
    "category": "method",
    "text": "num_additional_states(mechanism)\n\n\nReturn the dimension of the vector of additional states s (used for stateful contact models).\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.num_constraints-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_constraints",
    "category": "method",
    "text": "num_constraints(mechanism)\n\n\nReturn the number of constraints imposed by the mechanism\'s non-tree joints (i.e., the number of rows of the constraint Jacobian).\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.num_positions-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_positions",
    "category": "method",
    "text": "num_positions(mechanism)\n\n\nReturn the dimension of the joint configuration vector q.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.num_velocities-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "method",
    "text": "num_velocities(mechanism)\n\n\nReturn the dimension of the joint velocity vector v.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.out_joints-Tuple{RigidBody,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.out_joints",
    "category": "method",
    "text": "out_joints(body, mechanism)\n\n\nReturn the joints that have body as their predecessor.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.path-Tuple{Mechanism,RigidBody,RigidBody}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.path",
    "category": "method",
    "text": "path(mechanism, from, to)\n\n\nReturn the path from rigid body from to to along edges of the Mechanism\'s kinematic tree.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.predecessor-Tuple{Joint,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.predecessor",
    "category": "method",
    "text": "predecessor(joint, mechanism)\n\n\nReturn the body \'before\' the joint, i.e. the \'tail\' of the joint interpreted as an arrow in the Mechanism\'s kinematic graph.\n\nSee Joint.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.root_body-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.root_body",
    "category": "method",
    "text": "root_body(mechanism)\n\n\nReturn the root (stationary \'world\') body of the Mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.root_frame-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.root_frame",
    "category": "method",
    "text": "root_frame(mechanism)\n\n\nReturn the default frame of the root body.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.successor-Tuple{Joint,Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.successor",
    "category": "method",
    "text": "successor(joint, mechanism)\n\n\nReturn the body \'after\' the joint, i.e. the \'head\' of the joint interpreted as an arrow in the Mechanism\'s kinematic graph.\n\nSee Joint.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.tree_joints-Tuple{Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.tree_joints",
    "category": "method",
    "text": "tree_joints(mechanism)\n\n\nReturn the Joints that are part of the Mechanism\'s spanning tree as an iterable collection.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#RigidBodyDynamics.body_fixed_frame_definition-Tuple{Mechanism,CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.body_fixed_frame_definition",
    "category": "method",
    "text": "body_fixed_frame_definition(mechanism, frame)\n\n\nReturn the definition of body-fixed frame frame, i.e., the Transform3D from frame to the default frame of the body to which it is attached.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\nSee also default_frame, frame_definition.\n\n\n\n\n\n"
},

{
    "location": "mechanism/#Basic-functionality-1",
    "page": "Mechanism",
    "title": "Basic functionality",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism.jl\"]"
},

{
    "location": "mechanismstate/#",
    "page": "MechanismState",
    "title": "MechanismState",
    "category": "page",
    "text": ""
},

{
    "location": "mechanismstate/#MechanismState-1",
    "page": "MechanismState",
    "title": "MechanismState",
    "category": "section",
    "text": ""
},

{
    "location": "mechanismstate/#Index-1",
    "page": "MechanismState",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"mechanismstate.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.MechanismState",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.MechanismState",
    "category": "type",
    "text": "struct MechanismState{X, M, C, JointCollection}\n\nA MechanismState stores state information for an entire Mechanism. It contains the joint configuration and velocity vectors q and v, and a vector of additional states s. In addition, it stores cache variables that depend on q and v and are aimed at preventing double work.\n\nType parameters:\n\nX: the scalar type of the q, v, and s vectors.\nM: the scalar type of the Mechanism\nC: the scalar type of the cache variables (== promote_type(X, M))\nJointCollection: the type of the treejoints and nontreejoints members (a TypeSortedCollection subtype)\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#The-MechanismState-type-1",
    "page": "MechanismState",
    "title": "The MechanismState type",
    "category": "section",
    "text": "MechanismState"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.additional_state-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.additional_state",
    "category": "method",
    "text": "additional_state(state)\n\n\nReturn the vector of additional states s.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.bias_acceleration",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "function",
    "text": "bias_acceleration(state, body)\nbias_acceleration(state, body, safe)\n\n\nReturn the bias acceleration of the given body with respect to the world, i.e. the spatial acceleration of default_frame(body) with respect to the root frame of the mechanism, expressed in the root frame, when all joint accelerations are zero.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.bias_acceleration",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "function",
    "text": "bias_acceleration(state, joint)\nbias_acceleration(state, joint, safe)\n\n\nReturn the bias acceleration across the given joint, i.e. the spatial acceleration of frame_after(joint) with respect to frame_before(joint), expressed in the root frame of the mechanism when all joint accelerations are zero.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.configuration-Tuple{MechanismState,Union{JointID, #s148} where #s148<:Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration",
    "category": "method",
    "text": "configuration(state, joint)\n\n\nReturn the part of the configuration vector q associated with joint.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.configuration-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration",
    "category": "method",
    "text": "configuration(state)\n\n\nReturn the configuration vector q.\n\nNote that this returns a reference to the underlying data in state. The user is responsible for calling setdirty! after modifying this vector to ensure that dependent cache variables are invalidated.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.configuration_range-Tuple{MechanismState,Union{JointID, #s148} where #s148<:Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration_range",
    "category": "method",
    "text": "configuration_range(state, joint)\n\n\nReturn the range of indices into the joint configuration vector q corresponding to joint joint.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.crb_inertia",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.crb_inertia",
    "category": "function",
    "text": "crb_inertia(state, body)\ncrb_inertia(state, body, safe)\n\n\nReturn the composite rigid body inertia body expressed in the root frame of the mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.global_coordinates!-Tuple{MechanismState,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.global_coordinates!",
    "category": "method",
    "text": "global_coordinates!(state, q0, ϕ)\n\n\nConvert local coordinates phi centered around q_0 to (global) configuration vector q.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.gravitational_potential_energy",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.gravitational_potential_energy",
    "category": "function",
    "text": "gravitational_potential_energy(state, body)\ngravitational_potential_energy(state, body, safe)\n\n\nReturn the gravitational potential energy in the given state, computed as the negation of the dot product of the gravitational force and the center of mass expressed in the Mechanism\'s root frame.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.joint_transform",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.joint_transform",
    "category": "function",
    "text": "joint_transform(state, joint)\njoint_transform(state, joint, safe)\n\n\nReturn the joint transform for the given joint, i.e. the transform from frame_after(joint) to frame_before(joint).\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.local_coordinates!-Tuple{SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,MechanismState,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.local_coordinates!",
    "category": "method",
    "text": "local_coordinates!(ϕ, ϕd, state, q0)\n\n\nCompute local coordinates phi centered around (global) configuration vector q_0, as well as their time derivatives dotphi.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.normalize_configuration!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.normalize_configuration!",
    "category": "method",
    "text": "normalize_configuration!(state)\n\n\nProject the configuration vector q onto the configuration manifold.\n\nFor example:\n\nfor a part of q corresponding to a revolute joint, this method is a no-op;\nfor a part of q corresponding to a spherical joint that uses a unit quaternion\n\nto parameterize the orientation of its successor with respect to its predecessor, normalize_configuration! will renormalize the quaternion so that it is indeed of unit length.\n\nwarning: Warning\n\n\nThis method does not ensure that the configuration or velocity satisfy joint configuration or velocity limits/bounds.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.num_additional_states-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_additional_states",
    "category": "method",
    "text": "num_additional_states(state)\n\n\nReturn the length of the vector of additional states s (currently used for stateful contact models).\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.num_positions-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_positions",
    "category": "method",
    "text": "num_positions(state)\n\n\nReturn the length of the joint configuration vector q.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.num_velocities-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "method",
    "text": "num_velocities(state)\n\n\nReturn the length of the joint velocity vector v.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.principal_value!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.principal_value!",
    "category": "method",
    "text": "principal_value!(state)\n\n\nApplies the principalvalue functions from [Rotations.jl](https://github.com/FugroRoames/Rotations.jl/blob/d080990517f89b56c37962ad53a7fd24bd94b9f7/src/principalvalue.jl) to joint angles. This currently only applies to SPQuatFloating joints.\n\nFor example:\n\nfor a part of q corresponding to a revolute joint, this method is a no-op;\nfor a part of q corresponding to a SPQuatFloating joint this function applies\n\n`principal_value the orientation.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.rand_configuration!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.rand_configuration!",
    "category": "method",
    "text": "rand_configuration!(state)\n\n\nRandomize the configuration vector q. The distribution depends on the particular joint types present in the associated Mechanism. The resulting q is guaranteed to be on the Mechanism\'s configuration manifold. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.rand_velocity!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.rand_velocity!",
    "category": "method",
    "text": "rand_velocity!(state)\n\n\nRandomize the velocity vector v. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.relative_transform-Tuple{MechanismState,CartesianFrame3D,CartesianFrame3D}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_transform",
    "category": "method",
    "text": "relative_transform(state, from, to)\n\n\nReturn the homogeneous transform from from to to.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.relative_twist-Tuple{MechanismState,CartesianFrame3D,CartesianFrame3D}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_twist",
    "category": "method",
    "text": "relative_twist(state, body_frame, base_frame)\n\n\nReturn the twist of body_frame with respect to base_frame, expressed in the Mechanism\'s root frame.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.relative_twist-Tuple{MechanismState,Union{BodyID, #s147} where #s147<:RigidBody,Union{BodyID, #s144} where #s144<:RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_twist",
    "category": "method",
    "text": "relative_twist(state, body, base)\n\n\nReturn the twist of body with respect to base, expressed in the Mechanism\'s root frame.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.set_additional_state!-Tuple{MechanismState,AbstractArray{T,1} where T}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_additional_state!",
    "category": "method",
    "text": "set_additional_state!(state, s)\n\n\nSet the vector of additional states s.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.set_configuration!-Tuple{MechanismState,AbstractArray{T,1} where T}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_configuration!",
    "category": "method",
    "text": "set_configuration!(state, q)\n\n\nSet the configuration vector q. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.set_configuration!-Tuple{MechanismState,Joint,Any}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_configuration!",
    "category": "method",
    "text": "set_configuration!(state, joint, config)\n\n\nSet the part of the configuration vector associated with joint. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.set_velocity!-Tuple{MechanismState,AbstractArray{T,1} where T}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_velocity!",
    "category": "method",
    "text": "set_velocity!(state, v)\n\n\nSet the velocity vector v. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.set_velocity!-Tuple{MechanismState,Joint,Any}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_velocity!",
    "category": "method",
    "text": "set_velocity!(state, joint, vel)\n\n\nSet the part of the velocity vector associated with joint. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.setdirty!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.setdirty!",
    "category": "method",
    "text": "setdirty!(state)\n\n\nInvalidate all cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.spatial_inertia",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.spatial_inertia",
    "category": "function",
    "text": "spatial_inertia(state, body)\nspatial_inertia(state, body, safe)\n\n\nReturn the spatial inertia of body expressed in the root frame of the mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.transform_to_root",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.transform_to_root",
    "category": "function",
    "text": "transform_to_root(state, body)\ntransform_to_root(state, body, safe)\n\n\nReturn the transform from default_frame(body) to the root frame of the mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.twist_wrt_world",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.twist_wrt_world",
    "category": "function",
    "text": "twist_wrt_world(state, body)\ntwist_wrt_world(state, body, safe)\n\n\nReturn the twist of default_frame(body) with respect to the root frame of the mechanism, expressed in the root frame.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.velocity-Tuple{MechanismState,Union{JointID, #s148} where #s148<:Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity",
    "category": "method",
    "text": "velocity(state, joint)\n\n\nReturn the part of the velocity vector v associated with joint.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.velocity-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity",
    "category": "method",
    "text": "velocity(state)\n\n\nReturn the velocity vector v.\n\nNote that this function returns a read-write reference to a field in state. The user is responsible for calling setdirty! after modifying this vector to ensure that dependent cache variables are invalidated.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.velocity_range-Tuple{MechanismState,Union{JointID, #s148} where #s148<:Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity_range",
    "category": "method",
    "text": "velocity_range(state, joint)\n\n\nReturn the range of indices into the joint velocity vector v corresponding to joint joint.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.zero!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero!",
    "category": "method",
    "text": "zero!(state)\n\n\nZero both the configuration and velocity. Invalidates cache variables.\n\nSee zero_configuration!, zero_velocity!.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.zero_configuration!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero_configuration!",
    "category": "method",
    "text": "zero_configuration!(state)\n\n\n\'Zero\' the configuration vector q. Invalidates cache variables.\n\nNote that when the Mechanism contains e.g. quaternion-parameterized joints, q may not actually be set to all zeros; the quaternion part of the configuration vector would be set to identity. The contract is that each of the joint transforms should be an identity transform.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.zero_velocity!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero_velocity!",
    "category": "method",
    "text": "zero_velocity!(state)\n\n\nZero the velocity vector v. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#Base.copyto!-Tuple{AbstractArray{T,1} where T,MechanismState}",
    "page": "MechanismState",
    "title": "Base.copyto!",
    "category": "method",
    "text": "copyto!(dest, src)\n\n\nCopy state information in state dest to vector src (ordered [q; v; s]).\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#Base.copyto!-Tuple{MechanismState,AbstractArray{T,1} where T}",
    "page": "MechanismState",
    "title": "Base.copyto!",
    "category": "method",
    "text": "copyto!(dest, src)\n\n\nCopy state information in vector src (ordered [q; v; s]) to state dest.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#Base.copyto!-Tuple{MechanismState,MechanismState}",
    "page": "MechanismState",
    "title": "Base.copyto!",
    "category": "method",
    "text": "copyto!(dest, src)\n\n\nCopy (minimal representation of) state src to state dest.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#Random.rand!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "Random.rand!",
    "category": "method",
    "text": "rand!(state)\n\n\nRandomize both the configuration and velocity. Invalidates cache variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.configuration_index_to_joint_id-Tuple{MechanismState,Integer}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration_index_to_joint_id",
    "category": "method",
    "text": "configuration_index_to_joint_id(state, qindex)\n\n\nReturn the JointID of the joint associated with the given index into the configuration vector q.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.constraint_range-Tuple{MechanismState,Union{JointID, #s148} where #s148<:Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.constraint_range",
    "category": "method",
    "text": "constraint_range(state, joint)\n\n\nReturn the range of row indices into the constraint Jacobian corresponding to joint joint.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.reset_contact_state!-Tuple{MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.reset_contact_state!",
    "category": "method",
    "text": "reset_contact_state!(state)\n\n\nReset all contact state variables.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.supports-Tuple{Union{JointID, #s147} where #s147<:Joint,Union{BodyID, #s144} where #s144<:RigidBody,MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.supports",
    "category": "method",
    "text": "supports(joint, body, state)\n\n\nReturn whether joint supports body, i.e., joint is a tree joint on the path between body and the root.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.twist",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.twist",
    "category": "function",
    "text": "twist(state, joint)\ntwist(state, joint, safe)\n\n\nReturn the joint twist for the given joint, i.e. the twist of frame_after(joint) with respect to frame_before(joint), expressed in the root frame of the mechanism.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#RigidBodyDynamics.velocity_index_to_joint_id-Tuple{MechanismState,Integer}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity_index_to_joint_id",
    "category": "method",
    "text": "velocity_index_to_joint_id(state, qindex)\n\n\nReturn the JointID of the joint associated with the given index into the velocity vector v.\n\n\n\n\n\n"
},

{
    "location": "mechanismstate/#Functions-1",
    "page": "MechanismState",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_state.jl\"]"
},

{
    "location": "algorithms/#",
    "page": "Kinematics/dynamics algorithms",
    "title": "Kinematics/dynamics algorithms",
    "category": "page",
    "text": ""
},

{
    "location": "algorithms/#Algorithms-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Algorithms",
    "category": "section",
    "text": ""
},

{
    "location": "algorithms/#Index-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"algorithms.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "algorithms/#RigidBodyDynamics.DynamicsResult",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.DynamicsResult",
    "category": "type",
    "text": "mutable struct DynamicsResult{T, M}\n\nStores variables related to the dynamics of a Mechanism, e.g. the Mechanism\'s mass matrix and joint acceleration vector.\n\nType parameters:\n\nT: the scalar type of the dynamics-related variables.\nM: the scalar type of the Mechanism.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#The-DynamicsResult-type-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "The DynamicsResult type",
    "category": "section",
    "text": "DynamicsResult"
},

{
    "location": "algorithms/#RigidBodyDynamics.Spatial.center_of_mass-Tuple{MechanismState,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.Spatial.center_of_mass",
    "category": "method",
    "text": "center_of_mass(state, itr)\n\n\nCompute the center of mass of an iterable subset of a Mechanism\'s bodies in the given state. Ignores the root body of the mechanism.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.Spatial.center_of_mass-Tuple{MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.Spatial.center_of_mass",
    "category": "method",
    "text": "center_of_mass(state)\n\n\nCompute the center of mass of the whole Mechanism in the given state.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.dynamics!-Union{Tuple{X}, Tuple{DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M}, Tuple{DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractArray{T,1} where T}, Tuple{DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractArray{T,1} where T,AbstractDict{BodyID,#s141} where #s141<:Wrench}} where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics!",
    "category": "method",
    "text": "dynamics!(result, state)\ndynamics!(result, state, torques)\ndynamics!(result, state, torques, externalwrenches; stabilization_gains)\n\n\nCompute the joint acceleration vector dotv and Lagrange multipliers lambda that satisfy the joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau - K(q)^T lambda\n\nand the constraint equations\n\nK(q) dotv = -k\n\ngiven joint configuration vector q, joint velocity vector v, and (optionally) joint torques tau and external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism\'s bodies.\n\nThe stabilization_gains keyword argument can be used to set PD gains for Baumgarte stabilization, which can be used to prevent separation of non-tree (loop) joints. See Featherstone (2008), section 8.3 for more information. There are several options for specifying gains:\n\nnothing can be used to completely disable Baumgarte stabilization.\nGains can be specifed on a per-joint basis using any AbstractDict{JointID, <:RigidBodyDynamics.PDControl.SE3PDGains}, which maps the JointID for the non-tree joints of the mechanism to the gains for that joint.\nAs a special case of the second option, the same gains can be used for all joints by passing in a RigidBodyDynamics.CustomCollections.ConstDict{JointID}.\n\nThe default_constraint_stabilization_gains function is called to produce the default gains, which use the last option.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.dynamics!-Union{Tuple{X}, Tuple{Union{DenseArray{X,1}, ReinterpretArray{X,1,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, ReshapedArray{X,1,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, SubArray{X,1,A,I,L} where L where I<:Tuple{Vararg{Union{Int64, AbstractRange{Int64}, AbstractCartesianIndex},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, ReshapedArray{T,N,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, DenseArray}},DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractArray{X,1}}, Tuple{Union{DenseArray{X,1}, ReinterpretArray{X,1,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, ReshapedArray{X,1,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, SubArray{X,1,A,I,L} where L where I<:Tuple{Vararg{Union{Int64, AbstractRange{Int64}, AbstractCartesianIndex},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, ReshapedArray{T,N,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, DenseArray}},DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractArray{X,1},AbstractArray{T,1} where T}, Tuple{Union{DenseArray{X,1}, ReinterpretArray{X,1,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, ReshapedArray{X,1,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray}, SubArray{X,1,A,I,L} where L where I<:Tuple{Vararg{Union{Int64, AbstractRange{Int64}, AbstractCartesianIndex},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, ReshapedArray{T,N,A,MI} where MI<:Tuple{Vararg{SignedMultiplicativeInverse{Int64},N} where N} where A<:Union{ReinterpretArray{T,N,S,A} where S where A<:Union{SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, SubArray{T,N,A,I,true} where I<:Union{Tuple{Vararg{Real,N} where N}, Tuple{AbstractUnitRange,Vararg{Any,N} where N}} where A<:DenseArray where N where T, DenseArray} where N where T, DenseArray}},DynamicsResult,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractArray{X,1},AbstractArray{T,1} where T,AbstractDict{BodyID,#s142} where #s142<:Wrench}} where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics!",
    "category": "method",
    "text": "dynamics!(ẋ, result, state, x)\ndynamics!(ẋ, result, state, x, torques)\ndynamics!(ẋ, result, state, x, torques, externalwrenches; stabilization_gains)\n\n\nConvenience function for use with standard ODE integrators that takes a Vector argument\n\nx = left(beginarrayc\nq\nv\nendarrayright)\n\nand returns a Vector dotx.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.dynamics_bias!-Union{Tuple{X}, Tuple{SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,AbstractDict{BodyID,#s142} where #s142<:SpatialAcceleration,AbstractDict{BodyID,#s141} where #s141<:Wrench,MechanismState{X,M,C,JointCollection} where JointCollection where C where M}, Tuple{SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,AbstractDict{BodyID,#s140} where #s140<:SpatialAcceleration,AbstractDict{BodyID,#s139} where #s139<:Wrench,MechanismState{X,M,C,JointCollection} where JointCollection where C where M,AbstractDict{BodyID,#s17} where #s17<:Wrench}} where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics_bias!",
    "category": "method",
    "text": "dynamics_bias!(torques, biasaccelerations, wrenches, state)\ndynamics_bias!(torques, biasaccelerations, wrenches, state, externalwrenches)\n\n\nCompute the \'dynamics bias term\', i.e. the term\n\nc(q v w_textext)\n\nin the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism\'s bodies.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.dynamics_bias-Union{Tuple{MechanismState{X,M,C,JointCollection} where JointCollection where C}, Tuple{W}, Tuple{M}, Tuple{X}, Tuple{MechanismState{X,M,C,JointCollection} where JointCollection where C,AbstractDict{BodyID,Wrench{W}}}} where W where M where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics_bias",
    "category": "method",
    "text": "dynamics_bias(state)\ndynamics_bias(state, externalwrenches)\n\n\nCompute the \'dynamics bias term\', i.e. the term\n\nc(q v w_textext)\n\nin the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism\'s bodies.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.geometric_jacobian!-Tuple{GeometricJacobian,MechanismState,RigidBodyDynamics.Graphs.TreePath,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "method",
    "text": "geometric_jacobian!(jac, state, path, transformfun)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with a directed path in the Mechanism\'s spanning tree, (a collection of Joints and traversal directions) in the given state.\n\nA geometric Jacobian maps the Mechanism\'s joint velocity vector v to the twist of the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nSee also path, GeometricJacobian, Twist.\n\ntransformfun is a callable that may be used to transform the individual motion subspaces of each of the joints to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.geometric_jacobian!-Tuple{GeometricJacobian,MechanismState,RigidBodyDynamics.Graphs.TreePath,Transform3D}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "method",
    "text": "geometric_jacobian!(out, state, path, root_to_desired)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with a directed path in the Mechanism\'s spanning tree, (a collection of Joints and traversal directions) in the given state.\n\nA geometric Jacobian maps the Mechanism\'s joint velocity vector v to the twist of the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nSee also path, GeometricJacobian, Twist.\n\nroot_to_desired is the transform from the Mechanism\'s root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.geometric_jacobian!-Tuple{GeometricJacobian,MechanismState,RigidBodyDynamics.Graphs.TreePath}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "method",
    "text": "geometric_jacobian!(out, state, path)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with a directed path in the Mechanism\'s spanning tree, (a collection of Joints and traversal directions) in the given state.\n\nA geometric Jacobian maps the Mechanism\'s joint velocity vector v to the twist of the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nSee also path, GeometricJacobian, Twist.\n\nSee geometric_jacobian!(out, state, path, root_to_desired). Uses state to compute the transform from the Mechanism\'s root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.geometric_jacobian-Union{Tuple{C}, Tuple{M}, Tuple{X}, Tuple{MechanismState{X,M,C,JointCollection} where JointCollection,TreePath{RigidBody{M},Joint{M,JT} where JT<:JointType{M}}}} where C where M where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian",
    "category": "method",
    "text": "geometric_jacobian(state, path)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with a directed path in the Mechanism\'s spanning tree, (a collection of Joints and traversal directions) in the given state.\n\nA geometric Jacobian maps the Mechanism\'s joint velocity vector v to the twist of the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nSee also path, GeometricJacobian, Twist.\n\nThe Jacobian is computed in the Mechanism\'s root frame.\n\nSee geometric_jacobian!(out, state, path).\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.inverse_dynamics!-Union{Tuple{T}, Tuple{SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,AbstractDict{BodyID,Wrench{T}},AbstractDict{BodyID,SpatialAcceleration{T}},MechanismState,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T}, Tuple{SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,AbstractDict{BodyID,Wrench{T}},AbstractDict{BodyID,SpatialAcceleration{T}},MechanismState,SegmentedVector{JointID,T,KeyRange,P} where P<:AbstractArray{T,1} where KeyRange<:AbstractRange{JointID} where T,AbstractDict{BodyID,#s148} where #s148<:Wrench}} where T",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.inverse_dynamics!",
    "category": "method",
    "text": "inverse_dynamics!(torquesout, jointwrenchesout, accelerations, state, v̇)\ninverse_dynamics!(torquesout, jointwrenchesout, accelerations, state, v̇, externalwrenches)\n\n\nDo inverse dynamics, i.e. compute tau in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism\'s bodies.\n\nThis method implements the recursive Newton-Euler algorithm.\n\nCurrently doesn\'t support Mechanisms with cycles.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.inverse_dynamics-Union{Tuple{W}, Tuple{V}, Tuple{M}, Tuple{X}, Tuple{MechanismState{X,M,C,JointCollection} where JointCollection where C,SegmentedVector{JointID,V,KeyRange,P} where P<:AbstractArray{V,1} where KeyRange<:AbstractRange{JointID}}, Tuple{MechanismState{X,M,C,JointCollection} where JointCollection where C,SegmentedVector{JointID,V,KeyRange,P} where P<:AbstractArray{V,1} where KeyRange<:AbstractRange{JointID},AbstractDict{BodyID,Wrench{W}}}} where W where V where M where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.inverse_dynamics",
    "category": "method",
    "text": "inverse_dynamics(state, v̇)\ninverse_dynamics(state, v̇, externalwrenches)\n\n\nDo inverse dynamics, i.e. compute tau in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism\'s bodies.\n\nThis method implements the recursive Newton-Euler algorithm.\n\nCurrently doesn\'t support Mechanisms with cycles.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.mass-Tuple{Mechanism}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass",
    "category": "method",
    "text": "mass(m)\n\n\nReturn the total mass of the Mechanism.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.mass_matrix!-Tuple{LinearAlgebra.Symmetric,MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass_matrix!",
    "category": "method",
    "text": "mass_matrix!(M, state)\n\n\nCompute the joint-space mass matrix (also known as the inertia matrix) of the Mechanism in the given state, i.e., the matrix M(q) in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\nThis method implements the composite rigid body algorithm.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\nThe out argument must be an n_v times n_v lower triangular Symmetric matrix, where n_v is the dimension of the Mechanism\'s joint velocity vector v.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.mass_matrix-Union{Tuple{MechanismState{X,M,C,JointCollection} where JointCollection}, Tuple{C}, Tuple{M}, Tuple{X}} where C where M where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass_matrix",
    "category": "method",
    "text": "Compute the joint-space mass matrix (also known as the inertia matrix) of the Mechanism in the given state, i.e., the matrix M(q) in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\nThis method implements the composite rigid body algorithm.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.momentum_matrix!-Tuple{MomentumMatrix,MechanismState,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "method",
    "text": "momentum_matrix!(mat, state, transformfun)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism\'s joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism\'s joint velocity vector v.\n\ntransformfun is a callable that may be used to transform the individual momentum matrix blocks associated with each of the joints to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.momentum_matrix!-Tuple{MomentumMatrix,MechanismState,Transform3D}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "method",
    "text": "momentum_matrix!(mat, state, root_to_desired)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism\'s joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism\'s joint velocity vector v.\n\nroot_to_desired is the transform from the Mechanism\'s root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.momentum_matrix!-Tuple{MomentumMatrix,MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "method",
    "text": "momentum_matrix!(out, state)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism\'s joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism\'s joint velocity vector v.\n\nSee momentum_matrix!(out, state, root_to_desired). Uses state to compute the transform from the Mechanism\'s root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.momentum_matrix-Tuple{MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix",
    "category": "method",
    "text": "momentum_matrix(state)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism\'s joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nSee momentum_matrix!(out, state).\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.point_jacobian!-Tuple{PointJacobian,MechanismState,RigidBodyDynamics.Graphs.TreePath,Point3D}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.point_jacobian!",
    "category": "method",
    "text": "point_jacobian!(out, state, path, point)\n\n\nCompute the Jacobian mapping the Mechanism\'s joint velocity vector v to the velocity of a point fixed to the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nUses state to compute the transform from the Mechanism\'s root frame to the frame in which out is expressed if necessary.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.point_jacobian-Union{Tuple{C}, Tuple{M}, Tuple{X}, Tuple{MechanismState{X,M,C,JointCollection} where JointCollection,TreePath{RigidBody{M},Joint{M,JT} where JT<:JointType{M}},Point3D}} where C where M where X",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.point_jacobian",
    "category": "method",
    "text": "point_jacobian(state, path, point)\n\n\nCompute the Jacobian mapping the Mechanism\'s joint velocity vector v to the velocity of a point fixed to the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics._point_jacobian!-Tuple{PointJacobian,MechanismState,RigidBodyDynamics.Graphs.TreePath,Point3D,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics._point_jacobian!",
    "category": "method",
    "text": "_point_jacobian!(Jp, state, path, point, transformfun)\n\n\nCompute the Jacobian mapping the Mechanism\'s joint velocity vector v to the velocity of a point fixed to the target of the joint path (the body succeeding the last joint in the path) with respect to the source of the joint path (the body preceding the first joint in the path).\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.default_constraint_stabilization_gains-Union{Tuple{Type{T}}, Tuple{T}} where T",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.default_constraint_stabilization_gains",
    "category": "method",
    "text": "Return the default Baumgarte constraint stabilization gains. These gains result in critical damping, and correspond to T_stab = 01 in Featherstone (2008), section 8.3.\n\n\n\n\n\n"
},

{
    "location": "algorithms/#RigidBodyDynamics.subtree_mass-Union{Tuple{T}, Tuple{RigidBody{T},Mechanism{T}}} where T",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.subtree_mass",
    "category": "method",
    "text": "subtree_mass(base, mechanism)\n\n\nReturn the mass of a subtree of a Mechanism, rooted at base (including the mass of base).\n\n\n\n\n\n"
},

{
    "location": "algorithms/#Functions-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_algorithms.jl\"]"
},

{
    "location": "customcollections/#",
    "page": "Custom collection types",
    "title": "Custom collection types",
    "category": "page",
    "text": ""
},

{
    "location": "customcollections/#Custom-collection-types-1",
    "page": "Custom collection types",
    "title": "Custom collection types",
    "category": "section",
    "text": ""
},

{
    "location": "customcollections/#Index-1",
    "page": "Custom collection types",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"customcollections.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "customcollections/#Types-1",
    "page": "Custom collection types",
    "title": "Types",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics.CustomCollections]\nOrder   = [:type, :function]\nPages   = [\"custom_collections.jl\"]"
},

{
    "location": "caches/#",
    "page": "Cache types",
    "title": "Cache types",
    "category": "page",
    "text": ""
},

{
    "location": "caches/#RigidBodyDynamics.StateCache",
    "page": "Cache types",
    "title": "RigidBodyDynamics.StateCache",
    "category": "type",
    "text": "struct StateCache{M, JointCollection} <: RigidBodyDynamics.AbstractTypeDict\n\nA container that manages the creation and storage of MechanismState objects of various scalar types, associated with a given Mechanism.\n\nA StateCache can be used to write generic functions that use MechanismState objects, while avoiding overhead due to the construction of a new MechanismState with a given scalar type every time the function is called.\n\nExamples\n\njulia> mechanism = rand_tree_mechanism(Float64, Revolute{Float64}, Prismatic{Float64}, QuaternionFloating{Float64});\n\njulia> cache = StateCache(mechanism)\nStateCache{…}\n\njulia> state32 = cache[Float32]\nMechanismState{Float32, Float64, Float64, …}(…)\n\njulia> cache[Float32] === state32\ntrue\n\njulia> cache[Float64]\nMechanismState{Float64, Float64, Float64, …}(…)\n\n\n\n\n\n"
},

{
    "location": "caches/#RigidBodyDynamics.DynamicsResultCache",
    "page": "Cache types",
    "title": "RigidBodyDynamics.DynamicsResultCache",
    "category": "type",
    "text": "struct DynamicsResultCache{M} <: RigidBodyDynamics.AbstractTypeDict\n\nA container that manages the creation and storage of DynamicsResult objects of various scalar types, associated with a given Mechanism. Similar to StateCache.\n\n\n\n\n\n"
},

{
    "location": "caches/#RigidBodyDynamics.SegmentedVectorCache",
    "page": "Cache types",
    "title": "RigidBodyDynamics.SegmentedVectorCache",
    "category": "type",
    "text": "struct SegmentedVectorCache{K, KeyRange<:AbstractUnitRange{K}} <: RigidBodyDynamics.AbstractTypeDict\n\nA container that manages the creation and storage of heterogeneously typed SegmentedVector objects. Similar to StateCache.\n\n\n\n\n\n"
},

{
    "location": "caches/#StateCache-1",
    "page": "Cache types",
    "title": "StateCache",
    "category": "section",
    "text": "StateCache\nDynamicsResultCache\nSegmentedVectorCache"
},

{
    "location": "simulation/#",
    "page": "Simulation",
    "title": "Simulation",
    "category": "page",
    "text": ""
},

{
    "location": "simulation/#Simulation-1",
    "page": "Simulation",
    "title": "Simulation",
    "category": "section",
    "text": ""
},

{
    "location": "simulation/#Index-1",
    "page": "Simulation",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"simulation.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "simulation/#RigidBodyDynamics.simulate",
    "page": "Simulation",
    "title": "RigidBodyDynamics.simulate",
    "category": "function",
    "text": "simulate(state0, final_time)\nsimulate(state0, final_time, control!; Δt, stabilization_gains)\n\n\nBasic Mechanism simulation: integrate the state from time 0 to final_time starting from the initial state state0. Return a Vector of times, as well as Vectors of configuration vectors and velocity vectors at these times.\n\nOptionally, a function (or other callable) can be passed in as the third argument (control!). control! will be called at each time step of the simulation and allows you to specify joint torques given the time and the state of the Mechanism. It should look like this:\n\nfunction control!(torques::AbstractVector, t, state::MechanismState)\n    rand!(torques) # for example\nend\n\nThe integration time step can be specified using the Δt keyword argument (defaults to 1e-4).\n\nThe stabilization_gains keyword argument can be used to set PD gains for Baumgarte stabilization, which can be used to prevent separation of non-tree (loop) joints. See Featherstone (2008), section 8.3 for more information. There are several options for specifying gains:\n\nnothing can be used to completely disable Baumgarte stabilization.\nGains can be specifed on a per-joint basis using any AbstractDict{JointID, <:RigidBodyDynamics.PDControl.SE3PDGains}, which maps the JointID for the non-tree joints of the mechanism to the gains for that joint.\nAs a special case of the second option, the same gains can be used for all joints by passing in a RigidBodyDynamics.CustomCollections.ConstDict{JointID}.\n\nThe default_constraint_stabilization_gains function is called to produce the default gains, which use the last option.\n\nUses MuntheKaasIntegrator. See RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator for a lower level interface with more options.\n\n\n\n\n\n"
},

{
    "location": "simulation/#Basic-simulation-1",
    "page": "Simulation",
    "title": "Basic simulation",
    "category": "section",
    "text": "simulate"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "category": "type",
    "text": "struct MuntheKaasIntegrator{N, T, F, S<:OdeResultsSink, X, L, M<:(RigidBodyDynamics.OdeIntegrators.MuntheKaasStageCache{N,T,Q,V,S} where S<:(AbstractArray{T,1} where T) where V<:(AbstractArray{T,1} where T) where Q<:(AbstractArray{T,1} where T))}\n\nA Lie-group-aware ODE integrator.\n\nMuntheKaasIntegrator is used to properly integrate the dynamics of globally parameterized rigid joints (Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, 2006, Definition 2.9). Global parameterizations of e.g. SO(3) are needed to avoid singularities, but this leads to the problem that the tangent space no longer has the same dimension as the ambient space of the global parameterization. A Munthe-Kaas integrator solves this problem by converting back and forth between local and global coordinates at every integration time step.\n\nThe idea is to do the dynamics and compute the stages of the integration scheme in terms of local coordinates centered around the global parameterization of the configuration at the end of the previous time step (e.g. exponential coordinates), combine the stages into a new set of local coordinates as usual for Runge-Kutta methods, and then convert the local coordinates back to global coordinates.\n\nFrom Iserles et al., \'Lie-group methods\' (2000).\n\nAnother useful reference is Park and Chung, \'Geometric Integration on Euclidean Group with Application to Articulated Multibody Systems\' (2005).\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator-Union{Tuple{L}, Tuple{X}, Tuple{S}, Tuple{F}, Tuple{T}, Tuple{N}, Tuple{X,F,ButcherTableau{N,T,L},S}} where L where X where S<:OdeResultsSink where F where T where N",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "category": "method",
    "text": "MuntheKaasIntegrator(state, dynamics!, tableau, sink)\n\n\nCreate a MuntheKaasIntegrator given:\n\na callable dynamics!(vd, t, state) that updates the joint acceleration vector vd at time t and in state state;\na ButcherTableau tableau, specifying the integrator coefficients;\nan OdeResultsSink sink which processes the results of the integration procedure at each time step.\n\nstate must be of a type for which the following functions are defined:\n\nconfiguration(state), returns the configuration vector in global coordinates;\nvelocity(state), returns the velocity vector;\nadditional_state(state), returns the vector of additional states;\nset_velocity!(state, v), sets velocity vector to v;\nset_additional_state!(state, s), sets vector of additional states to s;\nglobal_coordinates!(state, q0, ϕ), sets global coordinates in state based on local coordinates ϕ centered around global coordinates q0;\nlocal_coordinates!(ϕ, ϕd, state, q0), converts state\'s global configuration q and velocity v to local coordinates centered around global coordinates q0.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.ButcherTableau",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.ButcherTableau",
    "category": "type",
    "text": "struct ButcherTableau{N, T, L}\n\nA Butcher tableau.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.OdeResultsSink",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.OdeResultsSink",
    "category": "type",
    "text": "abstract type OdeResultsSink\n\nDoes \'something\' with the results of an ODE integration (e.g. storing results, visualizing, etc.). Subtypes must implement:\n\ninitialize(sink, state): called with the initial state when integration begins.\nprocess(sink, t, state): called at every integration time step with the current state and time.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.RingBufferStorage",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.RingBufferStorage",
    "category": "type",
    "text": "mutable struct RingBufferStorage{T, Q<:(AbstractArray{T,1} where T), V<:(AbstractArray{T,1} where T)} <: OdeResultsSink\n\nAn OdeResultsSink that stores the state at each integration time step in a ring buffer.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.ExpandingStorage",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.ExpandingStorage",
    "category": "type",
    "text": "mutable struct ExpandingStorage{T, Q<:(AbstractArray{T,1} where T), V<:(AbstractArray{T,1} where T)} <: OdeResultsSink\n\nAn OdeResultsSink that stores the state at each integration time step in Vectors that may expand.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.integrate-Tuple{MuntheKaasIntegrator,Any,Any}",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.integrate",
    "category": "method",
    "text": "integrate(integrator, final_time, Δt; max_realtime_rate)\n\n\nIntegrate dynamics from the initial state at time 0 to final_time using step size Δt.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.runge_kutta_4-Union{Tuple{Type{T}}, Tuple{T}} where T",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.runge_kutta_4",
    "category": "method",
    "text": "Return the Butcher tableau for the standard fourth order Runge-Kutta integrator.\n\n\n\n\n\n"
},

{
    "location": "simulation/#RigidBodyDynamics.OdeIntegrators.step-Tuple{MuntheKaasIntegrator,Real,Real}",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.step",
    "category": "method",
    "text": "step(integrator, t, Δt)\n\n\nTake a single integration step.\n\n\n\n\n\n"
},

{
    "location": "simulation/#Lower-level-ODE-integration-interface-1",
    "page": "Simulation",
    "title": "Lower level ODE integration interface",
    "category": "section",
    "text": "MuntheKaasIntegrator\nMuntheKaasIntegrator(state::X, dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S) where {N, T, F, S<:OdeResultsSink, X, L}\nButcherTableau\nOdeResultsSink\nRingBufferStorage\nExpandingStorageModules = [RigidBodyDynamics.OdeIntegrators]\nOrder   = [:function]\nPages   = [\"ode_integrators.jl\"]"
},

{
    "location": "urdf/#",
    "page": "URDF parsing and writing",
    "title": "URDF parsing and writing",
    "category": "page",
    "text": ""
},

{
    "location": "urdf/#RigidBodyDynamics.URDF.default_urdf_joint_types",
    "page": "URDF parsing and writing",
    "title": "RigidBodyDynamics.URDF.default_urdf_joint_types",
    "category": "function",
    "text": "default_urdf_joint_types()\n\n\nDefault mapping from URDF joint type name to JointType subtype used by parse_urdf.\n\n\n\n\n\n"
},

{
    "location": "urdf/#RigidBodyDynamics.URDF.parse_urdf",
    "page": "URDF parsing and writing",
    "title": "RigidBodyDynamics.URDF.parse_urdf",
    "category": "function",
    "text": "parse_urdf(filename; scalar_type, floating, joint_types, root_joint_type, remove_fixed_tree_joints, gravity, revolute_joint_type, floating_joint_type)\n\n\nCreate a Mechanism by parsing a URDF file.\n\nKeyword arguments:\n\nscalar_type: the scalar type used to store the Mechanism\'s kinematic and inertial  properties. Default: Float64.\nfloating: whether to use a floating joint as the root joint. Default: false.\njoint_types: dictionary mapping URDF joint type names to JointType subtypes.  Default: default_urdf_joint_types().\nroot_joint_type: the JointType instance used to connect the parsed Mechanism to the world.  Default: an instance of the the joint type corresponding to the floating URDF joint type tag  if floating, otherwise in an instance of the joint type for the fixed URDF joint type tag.\nremove_fixed_tree_joints: whether to remove any fixed joints present in the kinematic tree  using remove_fixed_tree_joints!. Default: true.\ngravity: gravitational acceleration as a 3-vector expressed in the Mechanism\'s root frame  Default: [0.0, 0.0, -9.81].\n\n\n\n\n\n"
},

{
    "location": "urdf/#RigidBodyDynamics.URDF.write_urdf",
    "page": "URDF parsing and writing",
    "title": "RigidBodyDynamics.URDF.write_urdf",
    "category": "function",
    "text": "Serialize a Mechanism to the URDF file format.\n\nLimitations:\n\nfor <link> tags, only the <inertial> tag is written; there is no support for <visual> and <collision> tags.\nfor <joint> tags, only the <origin>, <parent>, <child>, and <limit> tags are written. There is no support for the <calibration> and <safety_controller> tags.\n\nThese limitations are simply due to the fact that Mechanisms do not store the required information to write these tags.\n\nKeyword arguments:\n\nrobot_name: used to set the name attribute of the root <robot> tag in the URDF. Default: nothing (name attribute will not be set).\ninclude_root: whether to include root_body(mechanism) in the URDF. If false, joints with root_body(mechanism) as their predecessor will also be omitted. Default: true.\n\n\n\n\n\n"
},

{
    "location": "urdf/#URDF-parsing-and-writing-1",
    "page": "URDF parsing and writing",
    "title": "URDF parsing and writing",
    "category": "section",
    "text": "default_urdf_joint_types\nparse_urdf\nwrite_urdf"
},

{
    "location": "benchmarks/#",
    "page": "Benchmarks",
    "title": "Benchmarks",
    "category": "page",
    "text": ""
},

{
    "location": "benchmarks/#Benchmarks-1",
    "page": "Benchmarks",
    "title": "Benchmarks",
    "category": "section",
    "text": "To attain maximal performance, it is recommended to pass -O3, --check-bounds=no as command line flags to julia. As of Julia 1.1, maximizing performance for the dynamics! algorithm requires either setting the number of BLAS threads to 1 (using LinearAlgebra; BLAS.set_num_threads(1)) if using OpenBLAS (the default), or compiling Julia with MKL. See this issue for more information.Run perf/runbenchmarks.jl to see benchmark results for the Atlas robot (v5). Results below are for the following scenarios:Compute the joint-space mass matrix.\nCompute both the mass matrix and a geometric Jacobian from the left hand to the right foot.\nDo inverse dynamics.\nDo forward dynamics.Note that results on Travis builds are not at all representative because of code coverage. Results on a reasonably fast machine at commit 32a2ccb:Output of versioninfo():Julia Version 1.1.0\nCommit 80516ca202 (2019-01-21 21:24 UTC)\nPlatform Info:\n  OS: Linux (x86_64-pc-linux-gnu)\n  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz\n  WORD_SIZE: 64\n  LIBM: libopenlibm\n  LLVM: libLLVM-6.0.1 (ORCJIT, broadwell)Mass matrix:  memory estimate:  0 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     5.379 μs (0.00% GC)\n  median time:      5.795 μs (0.00% GC)\n  mean time:        5.770 μs (0.00% GC)\n  maximum time:     9.258 μs (0.00% GC)Mass matrix and Jacobian from left hand to right foot:  memory estimate:  0 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     5.754 μs (0.00% GC)\n  median time:      5.856 μs (0.00% GC)\n  mean time:        5.940 μs (0.00% GC)\n  maximum time:     10.100 μs (0.00% GC)Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.Inverse dynamics:  memory estimate:  0 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     5.623 μs (0.00% GC)\n  median time:      5.727 μs (0.00% GC)\n  mean time:        5.886 μs (0.00% GC)\n  maximum time:     9.637 μs (0.00% GC)Forward dynamics:  memory estimate:  0 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     17.721 μs (0.00% GC)\n  median time:      18.862 μs (0.00% GC)\n  mean time:        18.755 μs (0.00% GC)\n  maximum time:     24.910 μs (0.00% GC)"
},

]}
