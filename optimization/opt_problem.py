from physics.collision.collisionHandler import collisionHandler
from optimization.SCP.SCvx import SCvx

class OptProblem():

    # TODO: -
    #

    def __init__(self):
        self.x0 = None  # initial state
        self.xf = None  # final state
        self.obs = None  # included obstacles
        self.robot = None  # used robot model
        self.par = None # parameter used in algorithm
        self.CHandler = None # Collision handler
        self.tf_min = None # minimal flight time
        self.tf_max = None # maximum flight time
        self.initial_x = None # initialization of states
        self.initial_u = None # initialization of actions
        self.initial_p = None # initialization of time_dilation

    def solve_problem(self):

        self.CHandler = collisionHandler(self.robot)
        self.CHandler.obs_data = self.obs

        if self.obs is not None:
            for obs in self.obs:
                self.CHandler.addObject(obs.type, obs.shape, obs.pos, obs.quat)

        scvx = SCvx(self.robot)
        solution = scvx.solve(self.x0,
                              self.xf,
                              self.tf_min,
                              self.tf_max,
                              self.initial_x,
                              self.initial_u,
                              self.initial_p,
                              self.par.num_time_steps,
                              self.par.max_num_iter,
                              self.CHandler)

        return solution

