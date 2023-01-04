from optimization import opt_utils as ou
from optimization.opt_problem import OptProblem
from optimization import problem_setups, initial_guess
from physics.multirotor_models.multirotor_double_integrator import QuadrotorDoubleInt
from physics.multirotor_models.multirotor_full_model import QuadrotorAutograd
from scripts.optimization.check.check_solution import check_solution
from visualization.sol_vis import visualize_solution
from visualization.ig_vis import visualize_initial_guess

# TODO: -
#

only_visualize = False

# choose which problem should be solved
prob = 2
if prob == 1:
    prob_setup = problem_setups.simple_flight_wo_obs()
    prob_name = "simple_flight_wo_obs"
elif prob == 2:
    prob_setup = problem_setups.complex_flight_spheres()
    prob_name = "complex_flight_spheres"
elif prob == 3:
    prob_setup = problem_setups.recovery_flight()
    prob_name = "recovery_flight"

# define optimization problem
optProb = OptProblem()

# define robot model
robot_type = "fM"
nr_motors = 4

if only_visualize == False:

    if robot_type == "dI":
        arm_length = 1e-4
        optProb.robot = QuadrotorDoubleInt(arm_length)

        # define start and end point
        optProb.x0 = prob_setup.x0_dI
        optProb.xf = prob_setup.xf_dI

    elif robot_type == "fM":
        arm_length = 0.046
        optProb.robot = QuadrotorAutograd(nr_motors, arm_length, prob_setup.t2w)

        # define start and end point
        optProb.x0 = prob_setup.x0_fM
        optProb.xf = prob_setup.xf_fM

    # define maximum and minimum flight time
    optProb.tf_min = prob_setup.tf_min
    optProb.tf_max = prob_setup.tf_max

    # define obstacles
    optProb.obs = prob_setup.obs

    # define number of timesteps
    t_steps_scvx = prob_setup.t_steps_scvx  # dI simple flight 30 ,full model only z 30

    optProb.initial_x, optProb.initial_u, optProb.initial_p = initial_guess.calc_initial_guess(optProb.robot,
                                                                                               t_steps_scvx,
                                                                                               prob_setup.noise,
                                                                                               optProb.xf,
                                                                                               optProb.x0,
                                                                                               optProb.tf_min,
                                                                                               optProb.tf_max,
                                                                                               "spline")

    # visalize initial guess
    visualize_initial_guess(optProb.initial_x[:3], optProb.xf, optProb.x0, optProb.obs)

    ######## solve problem with SCVX #######
    optProb.algorithm = "SCVX"

    # define SCVX Parameter
    par = ou.Parameter_scvx()
    if optProb.robot.type == "dI":
        par.max_num_iter = 50
    else:
        par.max_num_iter = 50
    par.num_time_steps = t_steps_scvx
    optProb.robot.dt = 1/(par.num_time_steps-1)

    # solve Problem with scvx
    optProb.par = par

    print("solving optimization problem with SCVX...")
    solution_scvx = optProb.solve_problem()

    print("solver time SCVX: {}".format(solution_scvx.time))

    print("checking SCVX solution ...")
    check, data_scvx, real_traj_scvx, int_error_scvx, int_error_scvx_small_dt, _, _ = check_solution(solution_scvx, optProb)

    print("SCVX solution correct?: {}".format(check))

    ou.save_opt_output(optProb,
                       prob_name,
                       solution_scvx,
                       data_scvx,
                       real_traj_scvx,
                       int_error_scvx,
                       int_error_scvx_small_dt)

sol_scvx = ou.load_opt_output(prob_name, robot_type, nr_motors)

visualize_solution(sol_scvx.robot,
                  sol_scvx.data,
                  sol_scvx.real_traj,
                  sol_scvx.obs,
                  sol_scvx.int_err,
                  sol_scvx.int_err_small_dt,
                  sol_scvx.x0, sol_scvx.xf, sol_scvx.t_dil)




