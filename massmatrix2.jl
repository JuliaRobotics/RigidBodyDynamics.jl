using BenchmarkTools
using RigidBodyDynamics
import RigidBodyDynamics: @nocachecheck, update_transforms!, update_crb_inertias!, cache_eltype, Graphs

mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 30]]...)
x = MechanismState{Float64}(mechanism)
rand!(x)

function mass_matrix2!(out::Symmetric, state::MechanismState)
    @boundscheck size(out, 1) == num_velocities(state) || error("mass matrix has wrong size")
    @boundscheck out.uplo == 'L' || error("expected a lower triangular symmetric matrix type as the mass matrix")
    fill!(out.data, 0)
    update_transforms!(state)
    update_crb_inertias!(state)
    joints = state.type_sorted_tree_joints
    foreach(joints) do jointi
        irange = velocity_range(state, jointi)
        bodyi = successor(jointi, state.mechanism)
        Si = motion_subspace(jointi, state.qs[jointi])
        @nocachecheck Si = transform(Si, transform_to_root(state, bodyi))
        @nocachecheck Ici = crb_inertia(state, bodyi)
        F = Ici * Si
        ancestor_joints = state.type_sorted_ancestor_joints[jointi]
        foreach(ancestor_joints) do jointj
            jrange = velocity_range(state, jointj)
            bodyj = successor(jointj, state.mechanism)
            Sj = motion_subspace(jointj, state.qs[jointj])
            @nocachecheck Sj = transform(Sj, transform_to_root(state, bodyj))
            block = F.angular' * Sj.angular + F.linear' * Sj.linear
            copy!(out.data, CartesianRange((irange, jrange)), block, CartesianRange(indices(block)))
        end
    end
    out
end

nv = num_velocities(mechanism)
M = Symmetric(Matrix{Float64}(nv, nv), :L);

mass_matrix2!(M, x)
setdirty!(x)
Profile.clear_malloc_data()
@benchmark (setdirty!($x); mass_matrix2!($M, $x))
