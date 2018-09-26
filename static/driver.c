// Standard headers
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Argument parsing
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Julia headers (for initialization and gc commands)
#include "uv.h"
#include "julia.h"

#ifdef JULIA_DEFINE_FAST_TLS // only available in Julia v0.7 and above
JULIA_DEFINE_FAST_TLS()
#endif

// Declare C prototype of Julia functions
extern jl_value_t* create_mechanism(char*, bool);
extern jl_value_t* create_state(jl_value_t*);
extern jl_value_t* create_dynamics_result(jl_value_t*);

extern double* inverse_dynamics(jl_value_t*, double*);
// extern int mass_matrix_benchmark(jl_array_t*);
// extern int dynamics_benchmark(jl_array_t*);

// main function (windows UTF16 -> UTF8 argument conversion code copied from julia's ui/repl.c)
int main(int argc, char *argv[])
{
    // Parse arguments.
    char *urdf = NULL;
    bool floating = false;
    char *csv = NULL;
    int c;
    opterr = 0;

    while ((c = getopt(argc, argv, "u:fc:")) != -1)
        switch (c)
        {
        case 'u':
            urdf = optarg;
            break;
        case 'f':
            floating = true;
            break;
        case 'c':
            csv = optarg;
            break;
        default:
            abort();
        }
    if (!urdf) {
        fprintf (stderr, "Must pass in URDF argument (-u).");
        abort();
    }
    if (!csv) {
        fprintf (stderr, "Must pass in CSV argument (-c).");
        abort();
    }

    int retcode;
    int i;
    // uv_setup_args(argc, argv); // no-op on Windows

    // initialization
    libsupport_init();

    // jl_options.compile_enabled = JL_OPTIONS_COMPILE_OFF; TODO
    // JULIAC_PROGRAM_LIBNAME defined on command-line for compilation
    jl_options.image_file = JULIAC_PROGRAM_LIBNAME;
    julia_init(JL_IMAGE_JULIA_HOME);

    // The following is not strictly necessary for this program, but can't hurt.

    // Initialize Core.ARGS with the full argv.
    jl_set_ARGS(argc, argv);

    // Set PROGRAM_FILE to argv[0].
    jl_set_global(jl_base_module,
        jl_symbol("PROGRAM_FILE"), (jl_value_t*)jl_cstr_to_string(argv[0]));

    // Set Base.ARGS to `String[ unsafe_string(argv[i]) for i = 1:argc ]`
    jl_array_t *ARGS = (jl_array_t*)jl_get_global(jl_base_module, jl_symbol("ARGS"));
    jl_array_grow_end(ARGS, argc - 1);
    for (i = 1; i < argc; i++) {
        jl_value_t *s = (jl_value_t*)jl_cstr_to_string(argv[i]);
        jl_arrayset(ARGS, s, i - 1);
    }

    jl_value_t *mechanism, *state, *result;

    // Parse URDF.
    mechanism = create_mechanism(urdf, floating);
    JL_GC_PUSH1(&mechanism);

    // Create MechanismState and DynamicsResult.
    state = create_state(mechanism);
    JL_GC_PUSH1(&state);

    result = create_dynamics_result(mechanism);

    // Get state dimensions
    jl_eval_string("using RigidBodyDynamics: num_positions, num_velocities");
    jl_function_t* num_positions = jl_get_function(jl_main_module, "num_positions");
    jl_function_t* num_velocities = jl_get_function(jl_main_module, "num_velocities");
    int nq = jl_unbox_int64(jl_call1(num_positions, mechanism));
    int nv = jl_unbox_int64(jl_call1(num_velocities, mechanism));

    // Allocate data arrays
    // jl_value_t *array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    // jl_array_t *q, *v, *vd, *tau;
    // q = jl_alloc_array_1d(array_type, nq);
    // v = jl_alloc_array_1d(array_type, nv);
    // vd = jl_alloc_array_1d(array_type, nv);
    // tau = jl_alloc_array_1d(array_type, nv);
    // FIXME: GC_PUSH?

    JL_GC_POP(); // mechanism



    // // call the work function, and get back a value
    // inverse_dynamics_benchmark(ARGS);
    // mass_matrix_benchmark(ARGS);
    // mass_matrix_benchmark(ARGS);

    // Clean up and gracefully exit
    jl_atexit_hook(retcode);
    return 0;
}
