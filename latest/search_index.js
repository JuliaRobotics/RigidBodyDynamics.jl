var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "index.html#RigidBodyDynamics-1",
    "page": "Home",
    "title": "RigidBodyDynamics",
    "category": "section",
    "text": "RigidBodyDynamics implements various rigid body dynamics and kinematics algorithms."
},

{
    "location": "index.html#Design-features-1",
    "page": "Home",
    "title": "Design features",
    "category": "section",
    "text": "Some of the key design features of this package are:pure Julia implementation, enabling seamless support for e.g. automatic differentiation using ForwardDiff.jl and symbolic dynamics using SymPy.jl.\neasy creation and modification of general rigid body mechanisms (including basic URDF parsing).\nextensive checks that verify that coordinate systems match before computation, with the goal of making reference frame mistakes impossible\nflexible caching of intermediate results to prevent doing double work\nfairly small codebase and few dependencies\nsingularity-free rotation parameterizations"
},

{
    "location": "index.html#Functionality-1",
    "page": "Home",
    "title": "Functionality",
    "category": "section",
    "text": "Current functionality of RigidBodyDynamics includes:kinematics/transforming points and free vectors from one coordinate system to another\ntransforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another\nrelative twists/spatial accelerations between bodies\nkinetic/potential energy\ncenter of mass\ngeometric/basic/spatial Jacobians\nmomentum\nmomentum matrix\nmomentum rate bias (= momentum matrix time derivative multiplied by joint velocity vector)\nmass matrix (composite rigid body algorithm)\ninverse dynamics (recursive Newton-Euler)\ndynamics\nsimulation, either using an off-the-shelf ODE integrator or using an included custom Munthe-Kaas integrator that properly handles second-order ODEs defined on a manifold.There is currently partial support for closed-loop systems (parallel mechanisms); more features will be added soon in this area. Contact is not yet supported."
},

{
    "location": "index.html#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Installing-Julia-1",
    "page": "Home",
    "title": "Installing Julia",
    "category": "section",
    "text": "Download links and more detailed instructions are available on the Julia website."
},

{
    "location": "index.html#Installing-RigidBodyDynamics-1",
    "page": "Home",
    "title": "Installing RigidBodyDynamics",
    "category": "section",
    "text": "To install the latest tagged release of RigidBodyDynamics, simply runPkg.add(\"RigidBodyDynamics\") To check out the master branch and work on the bleeding edge, additionally runPkg.checkout(\"RigidBodyDynamics\")RigidBodyDynamics currently only supports version 0.5 of Julia."
},

{
    "location": "index.html#About-1",
    "page": "Home",
    "title": "About",
    "category": "section",
    "text": "This library was inspired by IHMCRoboticsToolkit and by Drake.Most of the nomenclature used and algorithms implemented by this package stem from the following resources:Murray, Richard M., et al. A mathematical introduction to robotic manipulation. CRC press, 1994.\nFeatherstone, Roy. Rigid body dynamics algorithms. Springer, 2008.\nDuindam, Vincent. Port-based modeling and control for efficient bipedal walking robots. Diss. University of Twente, 2006."
},

{
    "location": "index.html#Related-packages-1",
    "page": "Home",
    "title": "Related packages",
    "category": "section",
    "text": "RigidBodyTreeInspector.jl - 3D visualization of RigidBodyDynamics.jl Mechanisms using Director."
},

{
    "location": "index.html#Contents-1",
    "page": "Home",
    "title": "Contents",
    "category": "section",
    "text": "Pages = [\n  \"quickstart.md\",\n  \"spatial.md\",\n  \"joints.md\",\n  \"rigidbody.md\",\n  \"mechanism.md\",\n  \"mechanismstate.md\",\n  \"algorithms.md\",\n  \"simulation.md\",\n  \"benchmarks.md\"]\nDepth = 2"
},

{
    "location": "index.html#Citing-this-library-1",
    "page": "Home",
    "title": "Citing this library",
    "category": "section",
    "text": "@misc{rigidbodydynamicsjl,\n author = \"Twan Koolen and contributors\",\n title = \"RigidBodyDynamics.jl\",\n year = 2016,\n url = \"https://github.com/tkoolen/RigidBodyDynamics.jl\"\n}"
},

{
    "location": "quickstart.html#",
    "page": "Quick start guide",
    "title": "Quick start guide",
    "category": "page",
    "text": ""
},

{
    "location": "quickstart.html#Quick-start-guide-1",
    "page": "Quick start guide",
    "title": "Quick start guide",
    "category": "section",
    "text": "To get started, see this Jupyter notebook.If you're interested in using different scalar types, see the symbolic double pendulum notebook."
},

{
    "location": "spatial.html#",
    "page": "Spatial vector algebra",
    "title": "Spatial vector algebra",
    "category": "page",
    "text": ""
},

{
    "location": "spatial.html#Spatial-vector-algebra-1",
    "page": "Spatial vector algebra",
    "title": "Spatial vector algebra",
    "category": "section",
    "text": ""
},

{
    "location": "spatial.html#Index-1",
    "page": "Spatial vector algebra",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"spatial.md\"]\nOrder   = [:type, :function, :macro]"
},

{
    "location": "spatial.html#Types-1",
    "page": "Spatial vector algebra",
    "title": "Types",
    "category": "section",
    "text": ""
},

{
    "location": "spatial.html#RigidBodyDynamics.CartesianFrame3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.CartesianFrame3D",
    "category": "Type",
    "text": "bitstype 64 CartesianFrame3D\n\nA CartesianFrame3D identifies a three-dimensional Cartesian coordinate system.\n\nCartesianFrame3Ds are typically used to annotate the frame in which certain quantities are expressed.\n\n\n\n"
},

{
    "location": "spatial.html#Coordinate-frames-1",
    "page": "Spatial vector algebra",
    "title": "Coordinate frames",
    "category": "section",
    "text": "CartesianFrame3D"
},

{
    "location": "spatial.html#RigidBodyDynamics.Transform3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Transform3D",
    "category": "Type",
    "text": "immutable Transform3D{T<:Number}\n\nA homogeneous transformation matrix representing the transformation from one three-dimensional Cartesian coordinate system to another.\n\n\n\n"
},

{
    "location": "spatial.html#Transforms-1",
    "page": "Spatial vector algebra",
    "title": "Transforms",
    "category": "section",
    "text": "Transform3D"
},

{
    "location": "spatial.html#RigidBodyDynamics.Point3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Point3D",
    "category": "Type",
    "text": "immutable Point3D{V<:AbstractArray{T,1}}\n\nA Point3D represents a position in a given coordinate system.\n\nA Point3D is a bound vector. Applying a Transform3D to a Point3D both rotates and translates the Point3D.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.FreeVector3D",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.FreeVector3D",
    "category": "Type",
    "text": "immutable FreeVector3D{V<:AbstractArray{T,1}}\n\nA FreeVector3D represents a free vector.\n\nExamples of free vectors include displacements and velocities of points.\n\nApplying a Transform3D to a FreeVector3D only rotates the FreeVector3D.\n\n\n\n"
},

{
    "location": "spatial.html#Points,-free-vectors-1",
    "page": "Spatial vector algebra",
    "title": "Points, free vectors",
    "category": "section",
    "text": "Point3D\nFreeVector3D"
},

{
    "location": "spatial.html#RigidBodyDynamics.SpatialInertia",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.SpatialInertia",
    "category": "Type",
    "text": "immutable SpatialInertia{T<:Number}\n\nA spatial inertia, or inertia matrix, represents the mass distribution of a rigid body.\n\nA spatial inertia expressed in frame i is defined as:\n\nI^i =\nint_Brholeft(xright)leftbeginarraycc\nhatp^Tleft(xright)hatpleft(xright)  hatpleft(xright)\nhatp^Tleft(xright)  I\nendarrayrightdx=leftbeginarraycc\nJ  hatc\nhatc^T  mI\nendarrayright\n\nwhere rho(x) is the density of point x, and p(x) are the coordinates of point x expressed in frame i. J is the mass moment of inertia, m is the total mass, and c is the 'cross part', center of mass position scaled by m.\n\n\n\n"
},

{
    "location": "spatial.html#Inertias-1",
    "page": "Spatial vector algebra",
    "title": "Inertias",
    "category": "section",
    "text": "SpatialInertia"
},

{
    "location": "spatial.html#RigidBodyDynamics.Twist",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Twist",
    "category": "Type",
    "text": "immutable Twist{T<:Number}\n\nA twist represents the relative angular and linear motion between two bodies.\n\nThe twist of frame j with respect to frame i, expressed in frame k is defined as\n\nT_j^ki=left(beginarrayc\nomega_j^ki\nv_j^ki\nendarrayright)inmathbbR^6\n\nsuch that\n\nleftbeginarraycc\nhatomega_j^ki  v_j^ki\n0  0\nendarrayright=H_i^kdotH_j^iH_k^j\n\nwhere H^beta_alpha is the homogeneous transform from frame alpha to frame beta, and hatx is the 3 times 3 skew symmetric matrix that satisfies hatx y = x times y for all y in mathbbR^3.\n\nHere, omega_j^ki is the angular part and v_j^ki is the linear part. Note that the linear part is not in general the same as the linear velocity of the origin of frame j.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.SpatialAcceleration",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.SpatialAcceleration",
    "category": "Type",
    "text": "immutable SpatialAcceleration{T<:Number}\n\nA spatial acceleration is the time derivative of a twist.\n\nSee Twist.\n\n\n\n"
},

{
    "location": "spatial.html#Twists,-spatial-accelerations-1",
    "page": "Spatial vector algebra",
    "title": "Twists, spatial accelerations",
    "category": "section",
    "text": "Twist\nSpatialAcceleration"
},

{
    "location": "spatial.html#RigidBodyDynamics.Momentum",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Momentum",
    "category": "Type",
    "text": "immutable Momentum{T<:Number}\n\nA Momentum is the product of a SpatialInertia and a Twist, i.e.\n\nh^i =\nleft(beginarrayc\nk^i\nl^i\nendarrayright) =\nI^i T^i j_k\n\nwhere I^i is the spatial inertia of a given body expressed in frame i, and T^i j_k is the twist of frame k (attached to the body) with respect to inertial frame j, expressed in frame i. k^i is the angular momentum and l^i is the linear momentum.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.Wrench",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.Wrench",
    "category": "Type",
    "text": "immutable Wrench{T<:Number}\n\nA wrench represents a system of forces.\n\nThe wrench w^i expressed in frame i is defined as\n\nw^i =\nleft(beginarrayc\ntau^i\nf^i\nendarrayright) =\nsum_jleft(beginarrayc\nr_j^itimes f_j^i\nf_j^i\nendarrayright)\n\nwhere the f_j^i are forces expressed in frame i, exerted at positions r_j^i. tau^i is the total torque and f^i is the total force.\n\n\n\n"
},

{
    "location": "spatial.html#Momenta,-wrenches-1",
    "page": "Spatial vector algebra",
    "title": "Momenta, wrenches",
    "category": "section",
    "text": "Momentum\nWrench"
},

{
    "location": "spatial.html#RigidBodyDynamics.GeometricJacobian",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.GeometricJacobian",
    "category": "Type",
    "text": "immutable GeometricJacobian{A<:AbstractArray{T,2}}\n\nA geometric Jacobian (also known as basic, or spatial Jacobian) maps a vector of joint velocities to a twist.\n\n\n\n"
},

{
    "location": "spatial.html#Geometric-Jacobians-1",
    "page": "Spatial vector algebra",
    "title": "Geometric Jacobians",
    "category": "section",
    "text": "GeometricJacobian"
},

{
    "location": "spatial.html#RigidBodyDynamics.MomentumMatrix",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.MomentumMatrix",
    "category": "Type",
    "text": "immutable MomentumMatrix{A<:AbstractArray{T,2}}\n\nA momentum matrix maps a joint velocity vector to momentum.\n\nThis is a slight generalization of the centroidal momentum matrix (Orin, Goswami, \"Centroidal momentum matrix of a humanoid robot: Structure and properties.\") in that the matrix (and hence the corresponding total momentum) need not be expressed in a centroidal frame.\n\n\n\n"
},

{
    "location": "spatial.html#Momentum-matrices-1",
    "page": "Spatial vector algebra",
    "title": "Momentum matrices",
    "category": "section",
    "text": "MomentumMatrix"
},

{
    "location": "spatial.html#RigidBodyDynamics.@framecheck",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.@framecheck",
    "category": "Macro",
    "text": "@framecheck(f1, f2)\n\n\nCheck that f1 and f2 are identical (when bounds checks are enabled).\n\nThrows an ArgumentError if f1 is not identical to f2 when bounds checks are enabled. @framecheck is a no-op when bounds checks are disabled.\n\n\n\n"
},

{
    "location": "spatial.html#The-@framecheck-macro-1",
    "page": "Spatial vector algebra",
    "title": "The @framecheck macro",
    "category": "section",
    "text": "@framecheck"
},

{
    "location": "spatial.html#RigidBodyDynamics.center_of_mass-Tuple{RigidBodyDynamics.SpatialInertia}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.center_of_mass",
    "category": "Method",
    "text": "center_of_mass(inertia)\n\n\nReturn the center of mass of the SpatialInertia as a Point3D.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.kinetic_energy-Tuple{RigidBodyDynamics.SpatialInertia,RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.kinetic_energy",
    "category": "Method",
    "text": "kinetic_energy(I, twist)\n\n\nCompute the kinetic energy of a body with spatial inertia I, which has twist T with respect to an inertial frame.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.newton_euler-Tuple{RigidBodyDynamics.SpatialInertia,RigidBodyDynamics.SpatialAcceleration,RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.newton_euler",
    "category": "Method",
    "text": "newton_euler(I, Ṫ, T)\n\n\nApply the Newton-Euler equations to find the external wrench required to make a body with spatial inertia I, which has twist T with respect to an inertial frame, achieve spatial acceleration dotT.\n\nThis wrench is also equal to the rate of change of momentum of the body.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.num_cols-Tuple{RigidBodyDynamics.GeometricJacobian}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.num_cols",
    "category": "Method",
    "text": "num_cols(jac)\n\n\nReturn the number of columns of the GeometricJacobian.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.point_velocity-Tuple{RigidBodyDynamics.Twist,RigidBodyDynamics.Point3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.point_velocity",
    "category": "Method",
    "text": "point_velocity(twist, point)\n\n\nGiven a twist T_j^ki of frame j with respect to frame i, expressed in frame k, and the location of a point fixed in frame j, also expressed in frame k, compute the velocity of the point relative to frame i.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.transform-Tuple{RigidBodyDynamics.GeometricJacobian,RigidBodyDynamics.Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.transform",
    "category": "Method",
    "text": "transform(jac, transform)\n\n\nTransform the GeometricJacobian to a different frame.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.transform-Tuple{RigidBodyDynamics.SpatialAcceleration,RigidBodyDynamics.Transform3D,RigidBodyDynamics.Twist,RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.transform",
    "category": "Method",
    "text": "transform(accel, oldToNew, twistOfCurrentWrtNew, twistOfBodyWrtBase)\n\n\nTransform the SpatialAcceleration to a different frame.\n\nThe transformation rule is obtained by differentiating the transformation rule for twists.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.transform-Tuple{RigidBodyDynamics.SpatialInertia{I},RigidBodyDynamics.Transform3D{T}}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.transform",
    "category": "Method",
    "text": "transform(inertia, t)\n\n\nTransform the SpatialInertia to a different frame.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.transform-Tuple{RigidBodyDynamics.Twist,RigidBodyDynamics.Transform3D}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.transform",
    "category": "Method",
    "text": "transform(twist, transform)\n\n\nTransform the Twist to a different frame.\n\n\n\n"
},

{
    "location": "spatial.html#Base.LinAlg.dot-Tuple{RigidBodyDynamics.Wrench,RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "Base.LinAlg.dot",
    "category": "Method",
    "text": "dot(w, t)\n\n\nCompute the mechanical power associated with a pairing of a wrench and a twist.\n\n\n\n"
},

{
    "location": "spatial.html#Base.exp-Tuple{RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "Base.exp",
    "category": "Method",
    "text": "exp(twist)\n\n\nConvert exponential coordinates to a homogeneous transform.\n\n\n\n"
},

{
    "location": "spatial.html#Base.log-Tuple{RigidBodyDynamics.Transform3D}",
    "page": "Spatial vector algebra",
    "title": "Base.log",
    "category": "Method",
    "text": "log(t)\n\n\nExpress a homogeneous transform in exponential coordinates centered around the identity.\n\n\n\n"
},

{
    "location": "spatial.html#RigidBodyDynamics.log_with_time_derivative-Tuple{RigidBodyDynamics.Transform3D,RigidBodyDynamics.Twist}",
    "page": "Spatial vector algebra",
    "title": "RigidBodyDynamics.log_with_time_derivative",
    "category": "Method",
    "text": "log_with_time_derivative(t, twist)\n\n\nCompute exponential coordinates as well as their time derivatives in one shot. This mainly exists because ForwardDiff won't work at the singularity of log. It is also ~50% faster than ForwardDiff in this case.\n\n\n\n"
},

{
    "location": "spatial.html#Functions-1",
    "page": "Spatial vector algebra",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"spatial.jl\", \"frames.jl\"]"
},

{
    "location": "joints.html#",
    "page": "Joints",
    "title": "Joints",
    "category": "page",
    "text": ""
},

{
    "location": "joints.html#Joints-1",
    "page": "Joints",
    "title": "Joints",
    "category": "section",
    "text": ""
},

{
    "location": "joints.html#Index-1",
    "page": "Joints",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"joints.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "joints.html#RigidBodyDynamics.Joint",
    "page": "Joints",
    "title": "RigidBodyDynamics.Joint",
    "category": "Type",
    "text": "type Joint{T<:Number}\n\nA joint represents a kinematic restriction of the relative twist between two rigid bodies to a linear subspace of dimension k. The state related to the joint is parameterized by two sets of variables, namely\n\na vector q in  mathcalQ, parameterizing the relative homogeneous transform.\na vector v in mathbbR^k, parameterizing the relative twist.\n\nA joint has a direction. The rigid body before the joint is called the joint's predecessor, and the rigid body after the joint is its successor.\n\nThe twist of the successor with respect to the predecessor is a linear function of v.\n\nFor some joint types (notably those using a redundant representation of relative orientation, such as a unit quaternion), dotq, the time derivative of q, may not be the same as v. However, an invertible linear transformation exists between dotq and v.\n\nSee also:\n\nDefinition 2.9 in Duindam, \"Port-Based Modeling and Control for Efficient Bipedal Walking Robots\", 2006.\nSection 4.4 of Featherstone, \"Rigid Body Dynamics Algorithms\", 2008.\n\n\n\n"
},

{
    "location": "joints.html#The-Joint-type-1",
    "page": "Joints",
    "title": "The Joint type",
    "category": "section",
    "text": "Joint"
},

{
    "location": "joints.html#RigidBodyDynamics.bias_acceleration-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{X,1},AbstractArray{X,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "Method",
    "text": "bias_acceleration(joint, q, v)\n\n\nReturn the acceleration of the joint's successor with respect to its predecessor in configuration q and at velocity v, when the joint acceleration dotv is zero.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.configuration_derivative_to_velocity!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.configuration_derivative_to_velocity!",
    "category": "Method",
    "text": "configuration_derivative_to_velocity!(joint, v, q, q̇)\n\n\nCompute joint velocity vector v given the joint configuration vector q and its time derivative dotq (in place).\n\nNote that this mapping is linear.\n\nSee also velocity_to_configuration_derivative!, the inverse mapping.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.constraint_wrench_subspace-Tuple{RigidBodyDynamics.Joint{M},RigidBodyDynamics.Transform3D{X}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.constraint_wrench_subspace",
    "category": "Method",
    "text": "constraint_wrench_subspace(joint, jointTransform)\n\n\nReturn a basis for the constraint wrench subspace of the joint, where jointTransform is the transform from the frame after the joint to the frame before the joint.\n\nThe constraint wrench subspace is a 6 times (6 - k) matrix, where k is the dimension of the velocity vector v, that maps a vector of Lagrange multipliers lambda to the constraint wrench exerted across the joint onto its successor.\n\nThe constraint wrench subspace is orthogonal to the motion subspace.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.global_coordinates!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.global_coordinates!",
    "category": "Method",
    "text": "global_coordinates!(joint, q, q0, ϕ)\n\n\nCompute the global parameterization of the joint's configuration, q, given a 'base' orientation q_0 and a vector of local coordinates  centered around q_0.\n\nSee also local_coordinates!.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.has_fixed_subspaces-Tuple{RigidBodyDynamics.Joint{M}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.has_fixed_subspaces",
    "category": "Method",
    "text": "has_fixed_subspaces(joint)\n\n\nWhether the joint's motion subspace and constraint wrench subspace depend on q.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.joint_torque!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1},AbstractArray{T,1},RigidBodyDynamics.Wrench}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_torque!",
    "category": "Method",
    "text": "joint_torque!(joint, τ, q, joint_wrench)\n\n\nGiven the wrench exerted across the joint on the joint's successor, compute the vector of joint torques tau (in place), in configuration q.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.joint_transform-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{X,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_transform",
    "category": "Method",
    "text": "joint_transform(joint, q)\n\n\nReturn a Transform3D representing the homogeneous transform from the frame after the joint to the frame before the joint for joint configuration vector q.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.local_coordinates!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.local_coordinates!",
    "category": "Method",
    "text": "local_coordinates!(joint, ϕ, ϕ̇, q0, q, v)\n\n\nCompute a vector of local coordinates phi around configuration q_0 corresponding to configuration q (in place). Also compute the time derivative dotphi of phi given the joint velocity vector v.\n\nThe local coordinate vector phi must be zero if and only if q = q_0.\n\nFor revolute or prismatic joint types, the local coordinates can just be phi = q - q_0, but for joint types with configuration vectors that are restricted to a manifold (e.g. when unit quaternions are used to represent orientation), elementwise subtraction may not make sense. For such joints, exponential coordinates could be used as the local coordinate vector phi.\n\nSee also global_coordinates!.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.motion_subspace-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{X,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.motion_subspace",
    "category": "Method",
    "text": "motion_subspace(joint, q)\n\n\nReturn a basis for the motion subspace of the joint in configuration q.\n\nThe motion subspace basis is a 6 times  k matrix, where k is the dimension of the velocity vector v, that maps v to the twist of the joint's successor with respect to its predecessor. The returned motion subspace is expressed in the frame after the joint, which is attached to the joint's successor.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.num_positions-Tuple{RigidBodyDynamics.Joint{M}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_positions",
    "category": "Method",
    "text": "num_positions(joint)\n\n\nReturn the length of the configuration vector of joint.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.num_velocities-Tuple{RigidBodyDynamics.Joint{M}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "Method",
    "text": "num_velocities(joint)\n\n\nReturn the length of the velocity vector of joint.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.rand_configuration!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.rand_configuration!",
    "category": "Method",
    "text": "rand_configuration!(joint, q)\n\n\nSet q to a random configuration. The distribution used depends on the joint type.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.velocity_to_configuration_derivative!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1},AbstractArray{T,1},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.velocity_to_configuration_derivative!",
    "category": "Method",
    "text": "velocity_to_configuration_derivative!(joint, q̇, q, v)\n\n\nCompute the time derivative dotq of the joint configuration vector q given q and the joint velocity vector v (in place).\n\nNote that this mapping is linear.\n\nSee also configuration_derivative_to_velocity!, the inverse mapping.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.zero_configuration!-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{T,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.zero_configuration!",
    "category": "Method",
    "text": "zero_configuration!(joint, q)\n\n\nSet q to the 'zero' configuration, corresponding to an identity joint transform.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.joint_twist-Tuple{RigidBodyDynamics.Joint{M},AbstractArray{X,1},AbstractArray{X,1}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.joint_twist",
    "category": "Method",
    "text": "joint_twist(joint, q, v)\n\n\nReturn the twist of joint's  successor with respect to its predecessor, expressed in the frame after the joint.\n\nNote that this is the same as Twist(motion_subspace(joint, q), v).\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.num_constraints-Tuple{RigidBodyDynamics.Joint}",
    "page": "Joints",
    "title": "RigidBodyDynamics.num_constraints",
    "category": "Method",
    "text": "num_constraints(joint)\n\n\nReturn the number of constraints imposed on the relative twist between the joint's predecessor and successor\n\n\n\n"
},

{
    "location": "joints.html#Functions-1",
    "page": "Joints",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"joint.jl\"]"
},

{
    "location": "joints.html#RigidBodyDynamics.Fixed",
    "page": "Joints",
    "title": "RigidBodyDynamics.Fixed",
    "category": "Type",
    "text": "immutable Fixed{T<:Number} <: RigidBodyDynamics.JointType{T<:Number}\n\nThe Fixed joint type is a degenerate joint type, in the sense that it allows no motion between its predecessor and successor rigid bodies.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.Prismatic",
    "page": "Joints",
    "title": "RigidBodyDynamics.Prismatic",
    "category": "Type",
    "text": "immutable Prismatic{T<:Number} <: RigidBodyDynamics.OneDegreeOfFreedomFixedAxis{T<:Number}\n\nA Prismatic joint type allows translation along a fixed axis.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.Prismatic-Tuple{StaticArrays.SVector{3,T}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.Prismatic",
    "category": "Method",
    "text": "Prismatic(axis)\n\n\nConstruct a new Prismatic joint type, allowing translation along axis (expressed in the frame before the joint).\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.QuaternionFloating",
    "page": "Joints",
    "title": "RigidBodyDynamics.QuaternionFloating",
    "category": "Type",
    "text": "immutable QuaternionFloating{T} <: RigidBodyDynamics.JointType{T}\n\nA floating joint type that uses a unit quaternion representation for orientation.\n\nFloating joints are 6-degree-of-freedom joints that are in a sense degenerate, as they impose no constraints on the relative motion between two bodies.\n\nThe full, 7-dimensional configuration vector of a QuaternionFloating joint type consists of a unit quaternion representing the orientation that rotates vectors from the frame 'directly after' the joint to the frame 'directly before' it, and a 3D position vector representing the origin of the frame after the joint in the frame before the joint.\n\nThe 6-dimensional velocity vector of a QuaternionFloating joint is the twist of the frame after the joint with respect to the frame before it, expressed in the frame after the joint.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.Revolute",
    "page": "Joints",
    "title": "RigidBodyDynamics.Revolute",
    "category": "Type",
    "text": "immutable Revolute{T<:Number} <: RigidBodyDynamics.OneDegreeOfFreedomFixedAxis{T<:Number}\n\nA Revolute joint type allows rotation about a fixed axis.\n\n\n\n"
},

{
    "location": "joints.html#RigidBodyDynamics.Revolute-Tuple{StaticArrays.SVector{3,T}}",
    "page": "Joints",
    "title": "RigidBodyDynamics.Revolute",
    "category": "Method",
    "text": "Revolute(axis)\n\n\nConstruct a new Revolute joint type, allowing rotation about axis (expressed in the frame before the joint).\n\n\n\n"
},

{
    "location": "joints.html#JointTypes-1",
    "page": "Joints",
    "title": "JointTypes",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:type, :function]\nPages   = [\"joint_types.jl\"]"
},

{
    "location": "rigidbody.html#",
    "page": "Rigid bodies",
    "title": "Rigid bodies",
    "category": "page",
    "text": ""
},

{
    "location": "rigidbody.html#Rigid-bodies-1",
    "page": "Rigid bodies",
    "title": "Rigid bodies",
    "category": "section",
    "text": ""
},

{
    "location": "rigidbody.html#Index-1",
    "page": "Rigid bodies",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"rigidbody.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.RigidBody",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.RigidBody",
    "category": "Type",
    "text": "type RigidBody{T<:Number}\n\nA non-deformable body.\n\nA RigidBody has inertia (represented as a SpatialInertia), unless it represents a root (world) body. A RigidBody additionally stores a list of definitions of coordinate systems that are rigidly attached to it.\n\n\n\n"
},

{
    "location": "rigidbody.html#The-RigidBody-type-1",
    "page": "Rigid bodies",
    "title": "The RigidBody type",
    "category": "section",
    "text": "RigidBody"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.add_contact_point!-Tuple{RigidBodyDynamics.RigidBody{T},RigidBodyDynamics.Contact.ContactPoint{T,RigidBodyDynamics.Contact.SoftContactModel{RigidBodyDynamics.Contact.HuntCrossleyModel{T},RigidBodyDynamics.Contact.ViscoelasticCoulombModel{T}}}}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.add_contact_point!",
    "category": "Method",
    "text": "add_contact_point!(body, point)\n\n\nAdd a new contact point to the rigid body\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.add_frame!-Tuple{RigidBodyDynamics.RigidBody{T},RigidBodyDynamics.Transform3D{T}}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.add_frame!",
    "category": "Method",
    "text": "add_frame!(body, transform)\n\n\nAdd a new frame definition to body, represented by a homogeneous transform from the CartesianFrame3D to be added to any other frame that is already attached to body.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.contact_points-Tuple{RigidBodyDynamics.RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.contact_points",
    "category": "Method",
    "text": "contact_points(body)\n\n\nReturn the contact points attached to the body as an ordered collection.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.default_frame-Tuple{RigidBodyDynamics.RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.default_frame",
    "category": "Method",
    "text": "default_frame(body)\n\n\nThe CartesianFrame3D with respect to which all other frames attached to body are defined.\n\nSee frame_definitions(body), frame_definition(body, frame).\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.fixed_transform-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.CartesianFrame3D,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.fixed_transform",
    "category": "Method",
    "text": "fixed_transform(body, from, to)\n\n\nReturn the transform from CartesianFrame3D from to to, both of which are rigidly attached to body.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.has_defined_inertia-Tuple{RigidBodyDynamics.RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.has_defined_inertia",
    "category": "Method",
    "text": "has_defined_inertia(b)\n\n\nWhether the body has a defined inertia.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.spatial_inertia!-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.SpatialInertia}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.spatial_inertia!",
    "category": "Method",
    "text": "spatial_inertia!(body, inertia)\n\n\nSet the spatial inertia of the body.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.spatial_inertia-Tuple{RigidBodyDynamics.RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.spatial_inertia",
    "category": "Method",
    "text": "spatial_inertia(b)\n\n\nReturn the spatial inertia of the body. If the inertia is undefined, calling this method will result in an error.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.change_default_frame!-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.change_default_frame!",
    "category": "Method",
    "text": "change_default_frame!(body, newDefaultFrame)\n\n\nChange the default frame of body to frame (which should already be among body's frame definitions).\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.frame_definition-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.frame_definition",
    "category": "Method",
    "text": "frame_definition(body, frame)\n\n\nReturn the Transform3D defining frame (attached to body) with respect to default_frame(body).\n\nThrows an error if frame is not attached to body.\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.frame_definitions-Tuple{RigidBodyDynamics.RigidBody}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.frame_definitions",
    "category": "Method",
    "text": "frame_definitions(body)\n\nReturn the list of homogeneous transforms (Transform3Ds) that define the coordinate systems attached to body with respect to a single common frame (default_frame(body)).\n\n\n\n"
},

{
    "location": "rigidbody.html#RigidBodyDynamics.is_fixed_to_body-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Rigid bodies",
    "title": "RigidBodyDynamics.is_fixed_to_body",
    "category": "Method",
    "text": "is_fixed_to_body(body, frame)\n\n\nWhether frame is attached to body (i.e. whether it is among frame_definitions(body)).\n\n\n\n"
},

{
    "location": "rigidbody.html#Functions-1",
    "page": "Rigid bodies",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"rigid_body.jl\"]"
},

{
    "location": "mechanism.html#",
    "page": "Mechanism",
    "title": "Mechanism",
    "category": "page",
    "text": ""
},

{
    "location": "mechanism.html#Mechanisms-1",
    "page": "Mechanism",
    "title": "Mechanisms",
    "category": "section",
    "text": ""
},

{
    "location": "mechanism.html#Index-1",
    "page": "Mechanism",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"mechanism.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "mechanism.html#RigidBodyDynamics.Mechanism",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.Mechanism",
    "category": "Type",
    "text": "type Mechanism{T<:Number}\n\nA Mechanism represents an interconnection of rigid bodies and joints. Mechanisms store the joint layout and inertia parameters, but no state-dependent information.\n\n\n\n"
},

{
    "location": "mechanism.html#The-Mechanism-type-1",
    "page": "Mechanism",
    "title": "The Mechanism type",
    "category": "section",
    "text": "Mechanism"
},

{
    "location": "mechanism.html#RigidBodyDynamics.parse_urdf",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.parse_urdf",
    "category": "Function",
    "text": "parse_urdf(scalartype, filename)\n\n\nCreate a Mechanism by parsing a URDF file.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.attach!",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.attach!",
    "category": "Function",
    "text": "attach!(mechanism, parentbody, childmechanism)\nattach!(mechanism, parentbody, childmechanism, childroot_to_parent)\n\n\nAttach a copy of childmechanism to mechanism. Return mappings from the bodies and joints of the childmechanism to the bodies and joints that were added to mechanism.\n\nEssentially replaces the root body of a copy of childmechanism with parentbody (which belongs to mechanism).\n\nNote: gravitational acceleration for childmechanism is ignored.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.attach!",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.attach!",
    "category": "Function",
    "text": "attach!(mechanism, predecessor, joint, jointToPredecessor, successor, successorToJoint)\n\nAttach successor to predecessor using joint.\n\nSee Joint for definitions of the terms successor and predecessor.\n\nThe Transform3Ds jointToPredecessor and successorToJoint define where joint is attached to each body. jointToPredecessor should define frame_before(joint) with respect to any frame fixed to predecessor, and likewise successorToJoint should define any frame fixed to successor with respect to frame_after(joint).\n\npredecessor is required to already be among the bodies of the Mechanism.\n\nIf successor is not yet a part of the Mechanism, it will be added to the Mechanism. Otherwise, the joint will be treated as a non-tree edge in the Mechanism, effectively creating a loop constraint that will be enforced using Lagrange multipliers (as opposed to using recursive algorithms).\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.maximal_coordinates-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.maximal_coordinates",
    "category": "Method",
    "text": "maximal_coordinates(mechanism)\n\n\nReturn a dynamically equivalent Mechanism, but with a flat tree structure with all bodies attached to the root body with a quaternion floating joint, and with the 'tree edge' joints of the input Mechanism transformed into non-tree edge joints (a constraint enforced using Lagrange multipliers in dynamics!). In addition, return:\n\na mapping from bodies in the maximal-coordinate Mechanism to their floating joints.\na mapping from bodies in the input Mechanism to bodies in the returned Mechanism\na mapping from joints in the input Mechanism to joints in the returned Mechanism\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.rand_chain_mechanism-Tuple{Type{T},Vararg{Any,N}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_chain_mechanism",
    "category": "Method",
    "text": "rand_chain_mechanism(t, jointTypes)\n\n\nCreate a random chain Mechanism with the given joint types.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.rand_floating_tree_mechanism-Tuple{Type{T},Vararg{Any,N}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_floating_tree_mechanism",
    "category": "Method",
    "text": "rand_floating_tree_mechanism(t, nonFloatingJointTypes)\n\n\nCreate a random tree Mechanism, with a quaternion floating joint as the first joint (between the root body and the first non-root body).\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.rand_tree_mechanism-Tuple{Type{T},Function,Vararg{Any,N}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_tree_mechanism",
    "category": "Method",
    "text": "rand_tree_mechanism(?, parentselector, jointTypes)\n\n\nCreate a random tree Mechanism with the given joint types. Each new body is attached to a parent selected using the parentselector function.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.rand_tree_mechanism-Tuple{Type{T},Vararg{Any,N}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rand_tree_mechanism",
    "category": "Method",
    "text": "rand_tree_mechanism(t, jointTypes)\n\n\nCreate a random tree Mechanism.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.remove_fixed_tree_joints!-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.remove_fixed_tree_joints!",
    "category": "Method",
    "text": "remove_fixed_tree_joints!(mechanism)\n\n\nRemove any fixed joints present as tree edges in mechanism by merging the rigid bodies that these fixed joints join together into bodies with equivalent inertial properties. Return the fixed joints that were removed.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.remove_joint!",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.remove_joint!",
    "category": "Function",
    "text": "remove_joint!(mechanism, joint, spanning_tree_next_edge)\nremove_joint!(mechanism, joint)\n\n\nRemove a joint from the mechanism. Rebuilds the spanning tree if the joint is part of the current spanning tree.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.submechanism-Tuple{RigidBodyDynamics.Mechanism{T},RigidBodyDynamics.RigidBody{T}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.submechanism",
    "category": "Method",
    "text": "submechanism(mechanism, submechanismroot)\n\n\nCreate a new Mechanism from the subtree of mechanism rooted at submechanismroot.\n\nAlso return mappings from the bodies and joints of the input mechanism to the bodies and joints of the submechanism.\n\nAny non-tree joint in mechanism will appear in the returned Mechanism if and only if both its successor and its predecessor are part of the subtree.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.rebuild_spanning_tree!",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.rebuild_spanning_tree!",
    "category": "Function",
    "text": "rebuild_spanning_tree!(mechanism, next_edge)\nrebuild_spanning_tree!(mechanism)\n\n\nReconstruct the mechanism's spanning tree.\n\n\n\n"
},

{
    "location": "mechanism.html#mechanism_create-1",
    "page": "Mechanism",
    "title": "Creating and modifying Mechanisms",
    "category": "section",
    "text": "parse_urdfModules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_manipulation.jl\"]"
},

{
    "location": "mechanism.html#RigidBodyDynamics.Graphs.path-Tuple{RigidBodyDynamics.Mechanism,RigidBodyDynamics.RigidBody,RigidBodyDynamics.RigidBody}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.Graphs.path",
    "category": "Method",
    "text": "path(mechanism, from, to)\n\n\nReturn the path from rigid body from to to along edges of the Mechanism's kinematic tree.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.bodies-Tuple{RigidBodyDynamics.Mechanism{T}}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.bodies",
    "category": "Method",
    "text": "bodies(mechanism)\n\n\nReturn the RigidBodys that are part of the Mechanism as an iterable collection.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.fixed_transform-Tuple{RigidBodyDynamics.Mechanism,RigidBodyDynamics.CartesianFrame3D,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.fixed_transform",
    "category": "Method",
    "text": "fixed_transform(mechanism, from, to)\n\n\nReturn the transform from CartesianFrame3D from to to, both of which are rigidly attached to the same RigidBody.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.in_joints-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.in_joints",
    "category": "Method",
    "text": "in_joints(body, mechanism)\n\n\nReturn the joints that have body as their successor.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.joint_to_parent-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joint_to_parent",
    "category": "Method",
    "text": "joint_to_parent(body, mechanism)\n\n\nReturn the joint that is part of the mechanism's kinematic tree and has body as its successor.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.joints-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joints",
    "category": "Method",
    "text": "joints(mechanism)\n\n\nReturn the Joints that are part of the Mechanism as an iterable collection.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.joints_to_children-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.joints_to_children",
    "category": "Method",
    "text": "joints_to_children(body, mechanism)\n\n\nReturn the joints that are part of the mechanism's kinematic tree and have body as their predecessor.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.num_additional_states-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_additional_states",
    "category": "Method",
    "text": "num_additional_states(mechanism)\n\n\nReturn the dimension of the vector of additional states s (used for stateful contact models).\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.num_positions-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_positions",
    "category": "Method",
    "text": "num_positions(mechanism)\n\n\nReturn the dimension of the joint configuration vector q.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.num_velocities-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "Method",
    "text": "num_velocities(mechanism)\n\n\nReturn the dimension of the joint velocity vector v.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.out_joints-Tuple{RigidBodyDynamics.RigidBody,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.out_joints",
    "category": "Method",
    "text": "out_joints(body, mechanism)\n\n\nReturn the joints that have body as their predecessor.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.predecessor-Tuple{RigidBodyDynamics.Joint,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.predecessor",
    "category": "Method",
    "text": "predecessor(joint, mechanism)\n\n\nReturn the body 'before' the joint, i.e. the 'tail' of the joint interpreted as an arrow in the Mechanism's kinematic graph.\n\nSee Joint.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.root_body-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.root_body",
    "category": "Method",
    "text": "root_body(mechanism)\n\n\nReturn the root (stationary 'world') body of the Mechanism.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.root_frame-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.root_frame",
    "category": "Method",
    "text": "root_frame(mechanism)\n\n\nReturn the default frame of the root body.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.successor-Tuple{RigidBodyDynamics.Joint,RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.successor",
    "category": "Method",
    "text": "successor(joint, mechanism)\n\n\nReturn the body 'after' the joint, i.e. the 'head' of the joint interpreted as an arrow in the Mechanism's kinematic graph.\n\nSee Joint.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.tree_joints-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.tree_joints",
    "category": "Method",
    "text": "tree_joints(mechanism)\n\n\nReturn the Joints that are part of the Mechanism's spanning tree as an iterable collection.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.body_fixed_frame_definition-Tuple{RigidBodyDynamics.Mechanism,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.body_fixed_frame_definition",
    "category": "Method",
    "text": "body_fixed_frame_definition(mechanism, frame)\n\n\nReturn the definition of body-fixed frame frame, i.e., the Transform3D from frame to the default frame of the body to which it is attached.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\nSee also default_frame, frame_definition.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.body_fixed_frame_to_body-Tuple{RigidBodyDynamics.Mechanism,RigidBodyDynamics.CartesianFrame3D}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.body_fixed_frame_to_body",
    "category": "Method",
    "text": "body_fixed_frame_to_body(mechanism, frame)\n\n\nReturn the RigidBody to which frame is attached.\n\nNote: this function is linear in the number of bodies and is not meant to be called in tight loops.\n\n\n\n"
},

{
    "location": "mechanism.html#RigidBodyDynamics.non_tree_joints-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Mechanism",
    "title": "RigidBodyDynamics.non_tree_joints",
    "category": "Method",
    "text": "non_tree_joints(mechanism)\n\n\nReturn the Joints that are not part of the Mechanism's spanning tree as an iterable collection.\n\n\n\n"
},

{
    "location": "mechanism.html#Basic-functionality-1",
    "page": "Mechanism",
    "title": "Basic functionality",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism.jl\"]"
},

{
    "location": "mechanismstate.html#",
    "page": "MechanismState",
    "title": "MechanismState",
    "category": "page",
    "text": ""
},

{
    "location": "mechanismstate.html#MechanismState-1",
    "page": "MechanismState",
    "title": "MechanismState",
    "category": "section",
    "text": ""
},

{
    "location": "mechanismstate.html#Index-1",
    "page": "MechanismState",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"mechanismstate.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.MechanismState",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.MechanismState",
    "category": "Type",
    "text": "immutable MechanismState{X<:Number, M<:Number, C<:Number}\n\nA MechanismState stores state information for an entire Mechanism. It contains the joint configuration and velocity vectors q and v, and a vector of additional states s. In addition, it stores cache variables that depend on q and v and are aimed at preventing double work.\n\nType parameters:\n\nX: the scalar type of the q, v, and s vectors.\nM: the scalar type of the Mechanism\nC: the scalar type of the cache variables (== promote_type(X, M))\n\n\n\n"
},

{
    "location": "mechanismstate.html#The-MechanismState-type-1",
    "page": "MechanismState",
    "title": "The MechanismState type",
    "category": "section",
    "text": "MechanismState"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.additional_state-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.additional_state",
    "category": "Method",
    "text": "additional_state(state)\n\n\nReturn the vector of additional states s.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.bias_acceleration-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "Method",
    "text": "bias_acceleration(state, joint)\n\n\nReturn the bias acceleration across the given joint, i.e. the spatial acceleration of frame_after(joint) with respect to frame_before(joint), expressed in the root frame of the mechanism when all joint accelerations are zero.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.bias_acceleration-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.bias_acceleration",
    "category": "Method",
    "text": "bias_acceleration(state, body)\n\n\nReturn the bias acceleration of the given body with respect to the world, i.e. the spatial acceleration of default_frame(body) with respect to the root frame of the mechanism, expressed in the root frame, when all joint accelerations are zero.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.configuration-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration",
    "category": "Method",
    "text": "configuration(state, joint)\n\n\nReturn the part of the configuration vector q associated with joint.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.configuration-Tuple{RigidBodyDynamics.MechanismState{X,M,C},RigidBodyDynamics.Graphs.TreePath{RigidBodyDynamics.RigidBody{M},RigidBodyDynamics.Joint{M}}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration",
    "category": "Method",
    "text": "configuration(state, path)\n\n\nReturn the part of the Mechanism's configuration vector q associated with the joints along path.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.configuration-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.configuration",
    "category": "Method",
    "text": "configuration(state)\n\n\nReturn the configuration vector q.\n\nNote that this returns a reference to the underlying data in state. The user is responsible for calling setdirty! after modifying this vector to ensure that dependent cache variables are invalidated.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.crb_inertia-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.crb_inertia",
    "category": "Method",
    "text": "crb_inertia(state, body)\n\n\nReturn the composite rigid body inertia body expressed in the root frame of the mechanism.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.global_coordinates!-Tuple{RigidBodyDynamics.MechanismState,Union{Base.ReshapedArray{T,1,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray{T,1},SubArray{T,1,A<:Union{Base.ReshapedArray{T,N,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray},I<:Tuple{Vararg{Union{Base.AbstractCartesianIndex,Colon,Int64,Range{Int64}},N}},L}},Union{Base.ReshapedArray{T,1,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray{T,1},SubArray{T,1,A<:Union{Base.ReshapedArray{T,N,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray},I<:Tuple{Vararg{Union{Base.AbstractCartesianIndex,Colon,Int64,Range{Int64}},N}},L}}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.global_coordinates!",
    "category": "Method",
    "text": "global_coordinates!(state, q0, ϕ)\n\n\nConvert local coordinates phi centered around q_0 to (global) configuration vector q.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.local_coordinates!-Tuple{RigidBodyDynamics.MechanismState,Union{Base.ReshapedArray{T,1,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray{T,1},SubArray{T,1,A<:Union{Base.ReshapedArray{T,N,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray},I<:Tuple{Vararg{Union{Base.AbstractCartesianIndex,Colon,Int64,Range{Int64}},N}},L}},Union{Base.ReshapedArray{T,1,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray{T,1},SubArray{T,1,A<:Union{Base.ReshapedArray{T,N,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray},I<:Tuple{Vararg{Union{Base.AbstractCartesianIndex,Colon,Int64,Range{Int64}},N}},L}},Union{Base.ReshapedArray{T,1,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray{T,1},SubArray{T,1,A<:Union{Base.ReshapedArray{T,N,A<:DenseArray,MI<:Tuple{Vararg{Base.MultiplicativeInverses.SignedMultiplicativeInverse{Int64},N}}},DenseArray},I<:Tuple{Vararg{Union{Base.AbstractCartesianIndex,Colon,Int64,Range{Int64}},N}},L}}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.local_coordinates!",
    "category": "Method",
    "text": "local_coordinates!(state, ϕ, ϕd, q0)\n\n\nCompute local coordinates phi centered around (global) configuration vector q_0, as well as their time derivatives dotphi.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.motion_subspace-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.motion_subspace",
    "category": "Method",
    "text": "motion_subspace(state, joint)\n\n\nReturn the motion subspace of the given joint expressed in frame_after(joint).\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.motion_subspace_in_world-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.motion_subspace_in_world",
    "category": "Method",
    "text": "motion_subspace_in_world(state, joint)\n\n\nReturn the motion subspace of the given joint expressed in the root frame of the mechanism.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.num_additional_states-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_additional_states",
    "category": "Method",
    "text": "num_additional_states(state)\n\n\nReturn the length of the vector of additional states s (currently used for stateful contact models).\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.num_positions-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_positions",
    "category": "Method",
    "text": "num_positions(state)\n\n\nReturn the length of the joint configuration vector q.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.num_velocities-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.num_velocities",
    "category": "Method",
    "text": "num_velocities(state)\n\n\nReturn the length of the joint velocity vector v.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.rand_configuration!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.rand_configuration!",
    "category": "Method",
    "text": "rand_configuration!(state)\n\n\nRandomize the configuration vector q. The distribution depends on the particular joint types present in the associated Mechanism. The resulting q is guaranteed to be on the Mechanism's configuration manifold. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.rand_velocity!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.rand_velocity!",
    "category": "Method",
    "text": "rand_velocity!(state)\n\n\nRandomize the velocity vector v. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.relative_transform-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.CartesianFrame3D,RigidBodyDynamics.CartesianFrame3D}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_transform",
    "category": "Method",
    "text": "relative_transform(state, from, to)\n\n\nReturn the homogeneous transform from from to to.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.relative_twist-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.CartesianFrame3D,RigidBodyDynamics.CartesianFrame3D}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_twist",
    "category": "Method",
    "text": "relative_twist(state, bodyFrame, baseFrame)\n\n\nReturn the twist of bodyFrame with respect to baseFrame, expressed in the Mechanism's root frame.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.relative_twist-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.relative_twist",
    "category": "Method",
    "text": "relative_twist(state, body, base)\n\n\nReturn the twist of body with respect to base, expressed in the Mechanism's root frame.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.set_additional_state!-Tuple{RigidBodyDynamics.MechanismState,AbstractArray{T,1}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_additional_state!",
    "category": "Method",
    "text": "set_additional_state!(state, s)\n\n\nSet the vector of additional states s.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.set_configuration!-Tuple{RigidBodyDynamics.MechanismState,AbstractArray{T,1}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_configuration!",
    "category": "Method",
    "text": "set_configuration!(state, q)\n\n\nSet the configuration vector q. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.set_configuration!-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint,AbstractArray{T,1}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_configuration!",
    "category": "Method",
    "text": "set_configuration!(state, joint, q)\n\n\nSet the part of the configuration vector associated with joint. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.set_velocity!-Tuple{RigidBodyDynamics.MechanismState,AbstractArray{T,1}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_velocity!",
    "category": "Method",
    "text": "set_velocity!(state, v)\n\n\nSet the velocity vector v. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.set_velocity!-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint,AbstractArray{T,1}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.set_velocity!",
    "category": "Method",
    "text": "set_velocity!(state, joint, v)\n\n\nSet the part of the velocity vector associated with joint. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.setdirty!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.setdirty!",
    "category": "Method",
    "text": "setdirty!(state)\n\n\nInvalidate all cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.spatial_inertia-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.spatial_inertia",
    "category": "Method",
    "text": "spatial_inertia(state, body)\n\n\nReturn the spatial inertia of body expressed in the root frame of the mechanism.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.transform-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.transform",
    "category": "Method",
    "text": "transform(state, joint)\n\n\nReturn the joint transform for the given joint, i.e. the transform from frame_after(joint) to frame_before(joint).\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.transform_to_root-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.transform_to_root",
    "category": "Method",
    "text": "transform_to_root(state, body)\n\n\nReturn the transform from default_frame(body) to the root frame of the mechanism.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.twist_wrt_world-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.twist_wrt_world",
    "category": "Method",
    "text": "twist_wrt_world(state, body)\n\n\nReturn the twist of default_frame(body) with respect to the root frame of the mechanism, expressed in the root frame.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.velocity-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity",
    "category": "Method",
    "text": "velocity(state, joint)\n\n\nReturn the part of the velocity vector v associated with joint.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.velocity-Tuple{RigidBodyDynamics.MechanismState{X,M,C},RigidBodyDynamics.Graphs.TreePath{RigidBodyDynamics.RigidBody{M},RigidBodyDynamics.Joint{M}}}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity",
    "category": "Method",
    "text": "velocity(state, path)\n\n\nReturn the part of the Mechanism's velocity vector v associated with the joints along path.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.velocity-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.velocity",
    "category": "Method",
    "text": "velocity(state)\n\n\nReturn the velocity vector v.\n\nNote that this function returns a read-write reference to a field in state. The user is responsible for calling setdirty! after modifying this vector to ensure that dependent cache variables are invalidated.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.zero!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero!",
    "category": "Method",
    "text": "zero!(state)\n\n\nZero both the configuration and velocity. Invalidates cache variables.\n\nSee zero_configuration!, zero_velocity!.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.zero_configuration!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero_configuration!",
    "category": "Method",
    "text": "zero_configuration!(state)\n\n\n'Zero' the configuration vector q. Invalidates cache variables.\n\nNote that when the Mechanism contains e.g. quaternion-parameterized joints, q may not actually be set to all zeros; the quaternion part of the configuration vector would be set to identity. The contract is that each of the joint transforms should be an identity transform.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.zero_velocity!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.zero_velocity!",
    "category": "Method",
    "text": "zero_velocity!(state)\n\n\nZero the velocity vector v. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#Base.Random.rand!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "Base.Random.rand!",
    "category": "Method",
    "text": "rand!(state)\n\n\nRandomize both the configuration and velocity. Invalidates cache variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.non_tree_joints-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.non_tree_joints",
    "category": "Method",
    "text": "non_tree_joints(state)\n\n\nReturn the Joints that are not part of the underlying Mechanism's spanning tree as an iterable collection.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.reset_contact_state!-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.reset_contact_state!",
    "category": "Method",
    "text": "reset_contact_state!(state)\n\n\nReset all contact state variables.\n\n\n\n"
},

{
    "location": "mechanismstate.html#RigidBodyDynamics.twist-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.Joint}",
    "page": "MechanismState",
    "title": "RigidBodyDynamics.twist",
    "category": "Method",
    "text": "twist(state, joint)\n\n\nReturn the joint twist for the given joint, i.e. the twist of frame_after(joint) with respect to frame_before(joint), expressed in the root frame of the mechanism.\n\n\n\n"
},

{
    "location": "mechanismstate.html#Functions-1",
    "page": "MechanismState",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_state.jl\"]"
},

{
    "location": "algorithms.html#",
    "page": "Kinematics/dynamics algorithms",
    "title": "Kinematics/dynamics algorithms",
    "category": "page",
    "text": ""
},

{
    "location": "algorithms.html#Algorithms-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Algorithms",
    "category": "section",
    "text": ""
},

{
    "location": "algorithms.html#Index-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"algorithms.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "algorithms.html#RigidBodyDynamics.DynamicsResult",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.DynamicsResult",
    "category": "Type",
    "text": "type DynamicsResult{M<:Number, T<:Number}\n\nStores variables related to the dynamics of a Mechanism, e.g. the Mechanism's mass matrix and joint acceleration vector.\n\nType parameters:\n\nM: the scalar type of the Mechanism.\nT: the scalar type of the dynamics-related variables.\n\n\n\n"
},

{
    "location": "algorithms.html#The-DynamicsResult-type-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "The DynamicsResult type",
    "category": "section",
    "text": "DynamicsResult"
},

{
    "location": "algorithms.html#RigidBodyDynamics.center_of_mass-Tuple{RigidBodyDynamics.MechanismState,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.center_of_mass",
    "category": "Method",
    "text": "center_of_mass(state, itr)\n\n\nCompute the center of mass of an iterable subset of a Mechanism's bodies in the given state. Ignores the root body of the mechanism.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.center_of_mass-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.center_of_mass",
    "category": "Method",
    "text": "center_of_mass(state)\n\n\nCompute the center of mass of the whole Mechanism in the given state.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.dynamics!",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics!",
    "category": "Function",
    "text": "dynamics!(ẋ, result, state, stateVec, torques)\ndynamics!(ẋ, result, state, stateVec, torques, externalwrenches)\ndynamics!(ẋ, result, state, stateVec)\n\n\nConvenience function for use with standard ODE integrators that takes a Vector argument\n\nx = left(beginarrayc\nq\nv\nendarrayright)\n\nand returns a Vector dotx.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.dynamics!",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics!",
    "category": "Function",
    "text": "dynamics!(result, state)\ndynamics!(result, state, torques, externalwrenches)\ndynamics!(result, state, torques)\n\n\nCompute the joint acceleration vector dotv and Lagrange multipliers lambda that satisfy the joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau - K(q)^T lambda\n\nand the constraint equations\n\nK(q) dotv = -k\n\ngiven joint configuration vector q, joint velocity vector v, and (optionally) joint torques tau and external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism's bodies. externalwrenches should be a vector for which externalwrenches[vertex_index(body)] is the wrench exerted upon body.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.geometric_jacobian!-Tuple{RigidBodyDynamics.GeometricJacobian,RigidBodyDynamics.MechanismState,RigidBodyDynamics.Graphs.TreePath,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "Method",
    "text": "geometric_jacobian!(out, state, path, transformfun)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with the joints that form a path in the Mechanism's spanning tree, in the given state.\n\nA geometric Jacobian maps the vector of velocities associated with the joint path to the twist of the body succeeding the last joint in the path with respect to the body preceding the first joint in the path.\n\nSee also path, GeometricJacobian, velocity(state, path), Twist.\n\ntransformfun is a callable that may be used to transform the individual motion subspaces of each of the joints to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.geometric_jacobian!-Tuple{RigidBodyDynamics.GeometricJacobian,RigidBodyDynamics.MechanismState,RigidBodyDynamics.Graphs.TreePath,RigidBodyDynamics.Transform3D}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "Method",
    "text": "geometric_jacobian!(out, state, path, root_to_desired)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with the joints that form a path in the Mechanism's spanning tree, in the given state.\n\nA geometric Jacobian maps the vector of velocities associated with the joint path to the twist of the body succeeding the last joint in the path with respect to the body preceding the first joint in the path.\n\nSee also path, GeometricJacobian, velocity(state, path), Twist.\n\nroot_to_desired is the transform from the Mechanism's root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.geometric_jacobian!-Tuple{RigidBodyDynamics.GeometricJacobian,RigidBodyDynamics.MechanismState,RigidBodyDynamics.Graphs.TreePath}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian!",
    "category": "Method",
    "text": "geometric_jacobian!(out, state, path)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with the joints that form a path in the Mechanism's spanning tree, in the given state.\n\nA geometric Jacobian maps the vector of velocities associated with the joint path to the twist of the body succeeding the last joint in the path with respect to the body preceding the first joint in the path.\n\nSee also path, GeometricJacobian, velocity(state, path), Twist.\n\nSee geometric_jacobian!(out, state, path, root_to_desired). Uses state to compute the transform from the Mechanism's root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.geometric_jacobian-Tuple{RigidBodyDynamics.MechanismState{X,M,C},RigidBodyDynamics.Graphs.TreePath{RigidBodyDynamics.RigidBody{M},RigidBodyDynamics.Joint{M}}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.geometric_jacobian",
    "category": "Method",
    "text": "geometric_jacobian(state, path)\n\n\nCompute a geometric Jacobian (also known as a basic, or spatial Jacobian) associated with the joints that form a path in the Mechanism's spanning tree, in the given state.\n\nA geometric Jacobian maps the vector of velocities associated with the joint path to the twist of the body succeeding the last joint in the path with respect to the body preceding the first joint in the path.\n\nSee also path, GeometricJacobian, velocity(state, path), Twist.\n\nThe Jacobian is computed in the Mechanism's root frame.\n\nSee geometric_jacobian!(out, state, path).\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.gravitational_potential_energy-Tuple{RigidBodyDynamics.MechanismState{X,M,C}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.gravitational_potential_energy",
    "category": "Method",
    "text": "gravitational_potential_energy(state)\n\n\nReturn the gravitational potential energy in the given state, computed as the negation of the dot product of the gravitational force and the center of mass expressed in the Mechanism's root frame.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.inverse_dynamics",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.inverse_dynamics",
    "category": "Function",
    "text": "inverse_dynamics(state, v̇, externalwrenches)\ninverse_dynamics(state, v̇)\n\n\nDo inverse dynamics, i.e. compute tau in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism's bodies. externalwrenches should be a vector for which externalwrenches[vertex_index(body)] is the wrench exerted upon body.\n\nThis method implements the recursive Newton-Euler algorithm.\n\nCurrently doesn't support Mechanisms with cycles.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.inverse_dynamics!",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.inverse_dynamics!",
    "category": "Function",
    "text": "inverse_dynamics!(torquesout, jointwrenchesout, accelerations, state, v̇)\ninverse_dynamics!(torquesout, jointwrenchesout, accelerations, state, v̇, externalwrenches)\n\n\nDo inverse dynamics, i.e. compute tau in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism's bodies. externalwrenches should be a vector for which externalwrenches[vertex_index(body)] is the wrench exerted upon body.\n\nThis method implements the recursive Newton-Euler algorithm.\n\nCurrently doesn't support Mechanisms with cycles.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.mass-Tuple{RigidBodyDynamics.Mechanism}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass",
    "category": "Method",
    "text": "mass(m)\n\n\nReturn the total mass of the Mechanism.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.mass_matrix!-Tuple{Symmetric{C,Array{C,2}},RigidBodyDynamics.MechanismState{X,M,C}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass_matrix!",
    "category": "Method",
    "text": "mass_matrix!(out, state)\n\n\nCompute the joint-space mass matrix (also known as the inertia matrix) of the Mechanism in the given state, i.e., the matrix M(q) in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\nThis method implements the composite rigid body algorithm.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\nThe out argument must be an n_v times n_v lower triangular Symmetric matrix, where n_v is the dimension of the Mechanism's joint velocity vector v.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.mass_matrix-Tuple{RigidBodyDynamics.MechanismState{X,M,C}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.mass_matrix",
    "category": "Method",
    "text": "mass_matrix(state)\n\n\nCompute the joint-space mass matrix (also known as the inertia matrix) of the Mechanism in the given state, i.e., the matrix M(q) in the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\nThis method implements the composite rigid body algorithm.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.momentum_matrix!-Tuple{RigidBodyDynamics.MomentumMatrix,RigidBodyDynamics.MechanismState,Any}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "Method",
    "text": "momentum_matrix!(out, state, transformfun)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism's joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism's joint velocity vector v.\n\ntransformfun is a callable that may be used to transform the individual momentum matrix blocks associated with each of the joints to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.momentum_matrix!-Tuple{RigidBodyDynamics.MomentumMatrix,RigidBodyDynamics.MechanismState,RigidBodyDynamics.Transform3D}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "Method",
    "text": "momentum_matrix!(out, state, root_to_desired)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism's joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism's joint velocity vector v.\n\nroot_to_desired is the transform from the Mechanism's root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.momentum_matrix!-Tuple{RigidBodyDynamics.MomentumMatrix,RigidBodyDynamics.MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix!",
    "category": "Method",
    "text": "momentum_matrix!(out, state)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism's joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nThe out argument must be a mutable MomentumMatrix with as many columns as the dimension of the Mechanism's joint velocity vector v.\n\nSee momentum_matrix!(out, state, root_to_desired). Uses state to compute the transform from the Mechanism's root frame to the frame in which out is expressed.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.momentum_matrix-Tuple{RigidBodyDynamics.MechanismState}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.momentum_matrix",
    "category": "Method",
    "text": "momentum_matrix(state)\n\n\nCompute the momentum matrix A(q) of the Mechanism in the given state.\n\nThe momentum matrix maps the Mechanism's joint velocity vector v to its total momentum.\n\nSee also MomentumMatrix.\n\nSee momentum_matrix!(out, state).\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.relative_acceleration-Tuple{RigidBodyDynamics.MechanismState,RigidBodyDynamics.RigidBody,RigidBodyDynamics.RigidBody,AbstractArray{T,1}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.relative_acceleration",
    "category": "Method",
    "text": "relative_acceleration(state, body, base, v̇)\n\n\nCompute the spatial acceleration of body with respect to base for the given state and joint acceleration vector dotv.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.dynamics_bias!",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.dynamics_bias!",
    "category": "Function",
    "text": "dynamics_bias!(torques, biasaccelerations, wrenches, state)\ndynamics_bias!(torques, biasaccelerations, wrenches, state, externalwrenches)\n\n\nCompute the 'dynamics bias term', i.e. the term\n\nc(q v w_textext)\n\nin the unconstrained joint-space equations of motion\n\nM(q) dotv + c(q v w_textext) = tau\n\ngiven joint configuration vector q, joint velocity vector v, joint acceleration vector dotv and (optionally) external wrenches w_textext.\n\nThe externalwrenches argument can be used to specify additional wrenches that act on the Mechanism's bodies. externalwrenches should be a vector for which externalwrenches[vertex_index(body)] is the wrench exerted upon body.\n\nThis method does its computation in place, performing no dynamic memory allocation.\n\n\n\n"
},

{
    "location": "algorithms.html#RigidBodyDynamics.subtree_mass-Tuple{RigidBodyDynamics.RigidBody{T},RigidBodyDynamics.Mechanism{T}}",
    "page": "Kinematics/dynamics algorithms",
    "title": "RigidBodyDynamics.subtree_mass",
    "category": "Method",
    "text": "subtree_mass(base, mechanism)\n\n\nReturn the mass of a subtree of a Mechanism, rooted at base (including the mass of base).\n\n\n\n"
},

{
    "location": "algorithms.html#Functions-1",
    "page": "Kinematics/dynamics algorithms",
    "title": "Functions",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics]\nOrder   = [:function]\nPages   = [\"mechanism_algorithms.jl\"]"
},

{
    "location": "simulation.html#",
    "page": "Simulation",
    "title": "Simulation",
    "category": "page",
    "text": ""
},

{
    "location": "simulation.html#Simulation-1",
    "page": "Simulation",
    "title": "Simulation",
    "category": "section",
    "text": ""
},

{
    "location": "simulation.html#Index-1",
    "page": "Simulation",
    "title": "Index",
    "category": "section",
    "text": "Pages   = [\"simulation.md\"]\nOrder   = [:type, :function]"
},

{
    "location": "simulation.html#RigidBodyDynamics.simulate",
    "page": "Simulation",
    "title": "RigidBodyDynamics.simulate",
    "category": "Function",
    "text": "simulate(state0, finalTime; Δt)\n\n\nBasic Mechanism simulation: integrate the state from time 0 to finalTime starting from the initial state state0. Return a Vector of times, as well as Vectors of configuration vectors and velocity vectors at these times.\n\nUses MuntheKaasIntegrator. See MuntheKaasIntegrator for a lower level interface with more options.\n\n\n\n"
},

{
    "location": "simulation.html#Basic-simulation-1",
    "page": "Simulation",
    "title": "Basic simulation",
    "category": "section",
    "text": "simulate"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.ExpandingStorage",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.ExpandingStorage",
    "category": "Type",
    "text": "type ExpandingStorage{T} <: RigidBodyDynamics.OdeIntegrators.OdeResultsSink\n\nAn OdeResultsSink that stores the state at each integration time step in Vectors that may expand.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "category": "Type",
    "text": "A Lie-group-aware ODE integrator.\n\nMuntheKaasIntegrator is used to properly integrate the dynamics of globally parameterized rigid joints (Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, 2006, Definition 2.9). Global parameterizations of e.g. SO(3) are needed to avoid singularities, but this leads to the problem that the tangent space no longer has the same dimension as the ambient space of the global parameterization. A Munthe-Kaas integrator solves this problem by converting back and forth between local and global coordinates at every integration time step.\n\nThe idea is to do the dynamics and compute the stages of the integration scheme in terms of local coordinates centered around the global parameterization of the configuration at the end of the previous time step (e.g. exponential coordinates), combine the stages into a new set of local coordinates as usual for Runge-Kutta methods, and then convert the local coordinates back to global coordinates.\n\nFrom Iserles et al., 'Lie-group methods' (2000).\n\nAnother useful reference is Park and Chung, 'Geometric Integration on Euclidean Group with Application to Articulated Multibody Systems' (2005).\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator-Tuple{F,RigidBodyDynamics.OdeIntegrators.ButcherTableau{N,T<:Number,L},S<:RigidBodyDynamics.OdeIntegrators.OdeResultsSink}",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator",
    "category": "Method",
    "text": "Create a MuntheKaasIntegrator given:\n\na callable dynamics!(vd, t, state) that updates the joint acceleration vector vd at time t and in state state;\na ButcherTableau tableau, specifying the integrator coefficients;\nan OdeResultsSink sink which processes the results of the integration procedure at each time step.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.OdeResultsSink",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.OdeResultsSink",
    "category": "Type",
    "text": "abstract OdeResultsSink\n\nDoes 'something' with the results of an ODE integration (e.g. storing results, visualizing, etc.). Subtypes must implement:\n\ninitialize(sink, state): called with the initial state when integration begins.\nprocess(sink, t, state): called at every integration time step with the current state and time.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.RingBufferStorage",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.RingBufferStorage",
    "category": "Type",
    "text": "type RingBufferStorage{T} <: RigidBodyDynamics.OdeIntegrators.OdeResultsSink\n\nAn OdeResultsSink that stores the state at each integration time step in a ring buffer.\n\n\n\n"
},

{
    "location": "simulation.html#Base.step-Tuple{RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator,Real,Any,Real}",
    "page": "Simulation",
    "title": "Base.step",
    "category": "Method",
    "text": "step(integrator, t, state, Δt)\n\n\nTake a single integration step.\n\nstate must be of a type for which the following functions are defined:\n\nconfiguration(state), returns the configuration vector in global coordinates;\nvelocity(state), returns the velocity vector;\nadditional_state(state), returns the vector of additional states;\nset_velocity!(state, v), sets velocity vector to v;\nset_additional_state!(state, s), sets vector of additional states to s;\nglobal_coordinates!(state, q0, ϕ), sets global coordinates in state based on local coordinates ϕ centered around global coordinates q0;\nlocal_coordinates!(state, ϕ, ϕd, q0), converts state's global configuration q and velocity v to local coordinates centered around global coordinates q0.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.integrate-Tuple{RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator,Any,Any,Any}",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.integrate",
    "category": "Method",
    "text": "integrate(integrator, state0, finalTime, Δt; maxRealtimeRate)\n\n\nIntegrate dynamics from the initial state state0 at time 0 to finalTime using step size Δt.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.runge_kutta_4-Tuple{Type{T}}",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.runge_kutta_4",
    "category": "Method",
    "text": "runge_kutta_4(scalartype)\n\n\nReturn the Butcher tableau for the standard fourth order Runge-Kutta integrator.\n\n\n\n"
},

{
    "location": "simulation.html#RigidBodyDynamics.OdeIntegrators.ButcherTableau",
    "page": "Simulation",
    "title": "RigidBodyDynamics.OdeIntegrators.ButcherTableau",
    "category": "Type",
    "text": "immutable ButcherTableau{N, T<:Number, L}\n\nA Butcher tableau.\n\n\n\n"
},

{
    "location": "simulation.html#Lower-level-ODE-integration-interface-1",
    "page": "Simulation",
    "title": "Lower level ODE integration interface",
    "category": "section",
    "text": "Modules = [RigidBodyDynamics.OdeIntegrators]\nOrder   = [:type, :function]\nPages   = [\"ode_integrators.jl\"]"
},

{
    "location": "benchmarks.html#",
    "page": "Benchmarks",
    "title": "Benchmarks",
    "category": "page",
    "text": ""
},

{
    "location": "benchmarks.html#Benchmarks-1",
    "page": "Benchmarks",
    "title": "Benchmarks",
    "category": "section",
    "text": "Run perf/runbenchmarks.jl (-O3 and --check-bounds=no flags recommended) to see benchmark results for the Atlas robot (v5) in the following scenarios:Compute the joint-space mass matrix.\nDo inverse dynamics.\nDo forward dynamics.Note that results on Travis builds are not at all representative because of code coverage. Results on a recent, fast machine with version 0.0.4:Output of versioninfo():Julia Version 0.5.0\nCommit 3c9d753 (2016-09-19 18:14 UTC)\nPlatform Info:\n  System: Linux (x86_64-pc-linux-gnu)\n  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz\n  WORD_SIZE: 64\n  BLAS: libopenblas (USE64BITINT DYNAMIC_ARCH NO_AFFINITY Haswell)\n  LAPACK: libopenblas64_\n  LIBM: libopenlibm\n  LLVM: libLLVM-3.7.1 (ORCJIT, broadwell)Mass matrix:  memory estimate:  0.00 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     23.034 μs (0.00% GC)\n  median time:      23.364 μs (0.00% GC)\n  mean time:        23.546 μs (0.00% GC)\n  maximum time:     52.605 μs (0.00% GC)\n  --------------\n  samples:          10000\n  evals/sample:     1\n  time tolerance:   5.00%\n  memory tolerance: 1.00%Inverse dynamics:  memory estimate:  0.00 bytes\n  allocs estimate:  0\n  --------------\n  minimum time:     29.178 μs (0.00% GC)\n  median time:      29.704 μs (0.00% GC)\n  mean time:        30.276 μs (0.00% GC)\n  maximum time:     65.232 μs (0.00% GC)\n  --------------\n  samples:          10000\n  evals/sample:     1\n  time tolerance:   5.00%\n  memory tolerance: 1.00%Forward dynamics:  memory estimate:  48.00 bytes\n  allocs estimate:  2\n  --------------\n  minimum time:     53.336 μs (0.00% GC)\n  median time:      82.928 μs (0.00% GC)\n  mean time:        83.334 μs (0.00% GC)\n  maximum time:     208.453 μs (0.00% GC)\n  --------------\n  samples:          10000\n  evals/sample:     1\n  time tolerance:   5.00%\n  memory tolerance: 1.00%"
},

]}
