from optimization import opt_utils as ou, problem_setups, initial_guess
from optimization.opt_problem import OptProblem
from physics.multirotor_models.multirotor_double_integrator import QuadrotorDoubleInt
from physics.multirotor_models.multirotor_full_model import QuadrotorAutograd
from optimization.check.check_solution import check_solution
from visualization.sol_vis import visualize_solution
from visualization.ig_vis import visualize_initial_guess
from visualization.anim_vis import animate_fM_meshcat
from visualization.anim_vis import animate_dI_meshcat

only_visualize = False

# choose which problem should be solved

prob_setup = problem_setups.flight_spheres()
prob_name = "flight_spheres"

# define robot model
robot_type = "fM"
n_motors = 4
arm_length = 0.046
thrust_to_weight = 1.2

if only_visualize == False:

    # define optimization problem
    optProb = OptProblem()
    optProb.robot = QuadrotorAutograd(n_motors, arm_length, thrust_to_weight)

    # define start and end point
    optProb.x0 = prob_setup.x0_fM
    optProb.xf = prob_setup.xf_fM

    # define maximum and minimum flight time
    optProb.tf_min = prob_setup.tf_min
    optProb.tf_max = prob_setup.tf_max

    # define obstacles
    optProb.obs = prob_setup.obs

    # define number of timesteps
    t_steps = prob_setup.t_steps

    optProb.initial_x, optProb.initial_u, optProb.initial_p = initial_guess.calc_initial_guess(robot = optProb.robot,
                                                                                               timesteps = t_steps,
                                                                                               noise_factor = 0.02,
                                                                                               xf = optProb.xf,
                                                                                               x0 = optProb.x0,
                                                                                               tf_min = optProb.tf_min,
                                                                                               tf_max = optProb.tf_max,
                                                                                               ig_type = "lin_interp",
                                                                                               visualize = True)


    # visalize initial guess
    visualize_initial_guess(optProb.initial_x[:,:3], optProb.xf, optProb.x0, optProb.obs)

    # solve problem with SCvx
    # define SCvx Parameter
    par = ou.Parameter_scvx()
    par.max_num_iter = 50
    par.num_time_steps = t_steps
    optProb.robot.dt = 1/(par.num_time_steps-1)

    # solve Problem with scvx
    optProb.par = par

    print("solving optimization problem with SCvx...")
    solution = optProb.solve_problem()

    print("solver time SCvx: {}".format(solution.time))

    print("checking SCvx solution ...")
    check, data, int_error, int_error_small_dt = check_solution(solution, optProb)

    print("SCvx solution correct?: {}".format(check))

    ou.save_opt_output(optProb,
                       prob_name,
                       solution,
                       data,
                       int_error,
                       int_error_small_dt)

sol = ou.load_opt_output(prob_name, robot_type, n_motors)

visualize_solution(sol.robot,
                   sol.data,
                   sol.obs,
                   sol.int_err,
                   sol.int_err_small_dt,
                   sol.t_dil)

if sol.robot.type == "fM":
    animate_fM_meshcat(sol.data,sol.obs)
else:
    animate_dI_meshcat(sol.data, sol.obs)




