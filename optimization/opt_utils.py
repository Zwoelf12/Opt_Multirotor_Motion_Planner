import numpy as np
import pickle
import rowan

class OptSolution():
    def __init__(self, states, actions, time=None, nu=None, opt_val=None, num_iter=None, tdil=None, time_cvxpy=None, time_cvx_solver=None, constr_viol = 0):
        self.states = states # optimal states
        self.actions = actions # optimal actions
        self.time = time # time needed to solve the problem
        self.nu = nu # dynamic constraint violation for scvx
        self.opt_val = opt_val # optimal problem value
        self.num_iter = num_iter # numbers of iterations until convergence
        self.time_dil = tdil # time dilation for timeoptimal calculation
        self.time_cvxpy = time_cvxpy # time spend in the cvxpy interface
        self.time_cvx_solver = time_cvx_solver # time taken only by the convex solver
        self.constr_viol = constr_viol # constraint violation of KOMO

class Obstacle():
    def __init__(self, type, shape, pos, quat):
        self.type = type # Obstacle type (box, sphere)
        self.shape = shape # shape of Obstacle
        self.pos = pos # position of Obstacle origin
        self.quat = quat # orientation of Obstacle

class Parameter_scvx():
    def __init__(self):
        self.max_num_iter = None # maximum number of iterations
        self.num_time_steps = None # number of time steps
        self.tf_min = None # minimum final time dilation
        self.tf_max = None # maximum final time dilation
        self.noise = None # noise used for initialization

class Parameter_KOMO():
    def __init__(self):
        self.phases = None # number of Phases the problem has
        self.time_steps_per_phase = None # number of time steps per phase
        self.time_per_phase = None # time per phase in seconds
        self.noise = None # noise used for initialization

class Opt_processData():
    def __init__(self):
        self.robot = None # used dynamic model
        self.data = None # extracted data
        self.real_traj = None # integrated data
        self.obs = None # list of obstacles
        self.int_err = None # integration error indicating dynamic violations
        self.int_err_small_dt = None # integration error indicating to small stepsize
        self.x0 = None # starting point
        self.xf = None # end point
        self.t_dil = None # optimized time dilation
        self.time = None # time needed to solve the nonconvex problem
        self.time_cvx_solver = None # time needed by the convex solver to solve the convex subproblem

def save_object(filename, sol):
    with open(filename, "wb") as outp:
        pickle.dump(sol, outp, pickle.HIGHEST_PROTOCOL)

def save_opt_output(optProb,
                    prob_name,
                    solution_scvx,
                    data_scvx,
                    int_error_scvx,
                    int_error_small_dt_scvx):

    opt_processData_scvx = Opt_processData()
    opt_processData_scvx.robot = optProb.robot
    opt_processData_scvx.data = data_scvx
    opt_processData_scvx.int_err = int_error_scvx
    opt_processData_scvx.int_err_small_dt = int_error_small_dt_scvx
    opt_processData_scvx.obs = optProb.obs
    opt_processData_scvx.x0 = optProb.x0
    opt_processData_scvx.xf = optProb.xf
    opt_processData_scvx.t_dil = solution_scvx.time_dil
    opt_processData_scvx.time = solution_scvx.time
    opt_processData_scvx.time_cvx_solver = solution_scvx.time_cvx_solver

    if optProb.robot.type == "dI":
        prob_name += "_dI"
    else:
        if optProb.robot.nrMotors == 2:
            prob_name += "_fM_2m"
        elif optProb.robot.nrMotors == 3:
            prob_name += "_fM_3m"
        elif optProb.robot.nrMotors == 4:
            prob_name += "_fM_4m"
        elif optProb.robot.nrMotors == 6:
            prob_name += "_fM_6m"
        else:
            prob_name += "_fM_8m"

    # save data
    path = "data/"

    filename = path + prob_name

    save_object(filename + "_scvx", opt_processData_scvx)

def load_object(filename):
    with open(filename, "rb") as input_file:
        sol = pickle.load(input_file)
    return sol

def load_opt_output(prob_name,robot_type, nrMotors):

    path = "data/"

    if robot_type == "dI":
        filename_scvx = prob_name + "_" + robot_type + "_scvx"
    else:
        filename_scvx = prob_name + "_" + robot_type + "_{}m".format(nrMotors) + "_scvx"

    sol_scvx = load_object(path + filename_scvx)

    return sol_scvx