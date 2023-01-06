import numpy as np
from optimization import opt_utils as ou

class Prob_setup():

    x0_dI = None
    xf_dI = None
    x0_fM = None
    xf_fM = None

    tf_min = None
    tf_max = None

    t_steps_scvx = None

    obs = None

    t2w = 1.4

    noise = 0.01

########### define motion planning problems here #############
def flight_spheres():

    setup = Prob_setup()

    setup.x0_dI = np.array([0.1, -1.3, 1,
                            0, 0, 0], dtype=np.float64)

    setup.xf_dI = np.array([-0.1, 1.3, 1,
                            0, 0, 0], dtype=np.float64)

    setup.x0_fM = np.array([0.1, -1.3, 1.,
                            0., 0., 0.,
                            1., 0., 0., 0.,
                            0., 0., 0.],
                           dtype=np.float64)  # from -0.5 to 0.5 and 0.5 hight diff works for full model # 0.71,0.71 for 90°

    setup.xf_fM = np.array([-0.1, 1.3, 1.,
                            0., 0., 0.,
                            1., 0., 0., 0.,
                            0., 0., 0.], dtype=np.float64)

    setup.tf_min = 2.7
    setup.tf_max = 2.7

    setup.t_steps = 30

    obs1 = ou.Obstacle("sphere", [0.4], [0., -0.7, 1.], [1, 0, 0, 0])
    obs2 = ou.Obstacle("sphere", [0.4], [1., 0., 1.], [1, 0, 0, 0])
    obs3 = ou.Obstacle("sphere", [0.4], [-1., 0., 1.], [1, 0, 0, 0])
    obs4 = ou.Obstacle("sphere", [0.4], [0., 0., 1.7], [1, 0, 0, 0])
    obs5 = ou.Obstacle("sphere", [0.4], [0., 0., .3], [1, 0, 0, 0])
    obs6 = ou.Obstacle("sphere", [0.4], [0., 0.7, 1.], [1, 0, 0, 0])
    setup.obs = [obs1, obs2, obs3, obs4, obs5, obs6]

    return setup

    
    
