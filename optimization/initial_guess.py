import numpy as np
import scipy.interpolate as si
import rowan

def calc_initial_guess(robot, timesteps, noise_factor, xf, x0, tf_min, tf_max, ig_type = "lin_interp", visualize = False):

    T = timesteps

    # calculate ranges
    state_range = robot.max_x - robot.min_x
    input_range = robot.max_u - robot.min_u

    stateDim = state_range.shape[0]
    actionDim = input_range.shape[0]

    # set up initial trajectory
    initial_x = np.zeros((T, stateDim))
    initial_u = np.zeros((T, actionDim))
    # initial time dilation
    initial_p = (tf_min + tf_max) / 2

    # calculate geometric initial guess
    if ig_type == "lin_interp":
        # use interpolation between start and end point as initial guess
        initial_x[:, 0:3] += np.linspace(x0[0:3], xf[0:3], T)

        # add noise depending on the state and input range
        # pos and vel
        initial_x[1:-1, :6] += np.random.normal(0, state_range[:6] * noise_factor, initial_x[:, :6].shape)[1:-1]

    elif ig_type == "spline":
        # use spline interpolation
        initial_x[:, 0:3] += spline_initial_guess(x0[:3], xf[:3], T, n_control_points=5, degree=3, noise_fac= noise_factor) # noise factor is now not depending on the limits anymore


    if robot.type == "fM":
        # interpolate quaternions
        q0 = rowan.normalize(x0[6:10])
        qf = rowan.normalize(xf[6:10])
        initial_x[:, 6:10] += rowan.interpolate.slerp(q0, qf, np.linspace(0, 1, T))

        # normalize quaternions
        Q = initial_x[:, 6:10]
        Q_norm = np.linalg.norm(initial_x[:, 6:10], axis=1)
        initial_x[:, 6:10] = (Q.T / Q_norm).T

        # add gravity compensation
        initial_u[:, :] += (9.81 * robot.mass) / robot.nrMotors

    # add noise depending on the state and input range

    if robot.type == "fM":
        # orientation
        orient_noise_euler = np.random.normal(0, 2*np.pi * noise_factor, (T,3))
        orient_noise_quat = rowan.from_euler(orient_noise_euler[:,0], orient_noise_euler[:,1], orient_noise_euler[:,2])
        initial_x[1:-1, 6:10] += orient_noise_quat[1:-1]

        # rotational velocities
        initial_x[1:-1, 10:] += np.random.normal(0, state_range[10:] * noise_factor, initial_x[:, 10:].shape)[1:-1]

    # input
    initial_u += np.random.normal(0., input_range * noise_factor, initial_u.shape)

    # time dilation
    initial_p += np.random.normal(initial_p, noise_factor, 1)

    return initial_x, initial_u, initial_p

def bspline(cv, n=100, degree=9):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
                  False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)

    degree = np.clip(degree,1,count-1)

    # Calculate knot vector
    kv = None
    kv = np.concatenate(([0]*degree, np.arange(count-degree+1), [count-degree]*degree))

    # Calculate query range
    u = np.linspace(0,(count-degree),n)

    # Calculate result
    return np.array(si.splev(u, (kv,cv.T,degree))).T

def spline_initial_guess(x0, xf, ntimesteps, n_control_points = 5, degree = 3, noise_fac = 0.5):

    # use interpolation as initialization for control vector
    lin_interp = np.linspace(x0, xf, n_control_points)

    # add noise to control vector
    lin_interp[1:-1] = lin_interp[1:-1] + np.random.normal(0, noise_fac, lin_interp[1:-1].shape)

    knots = lin_interp

    spline = bspline(knots,ntimesteps,degree)

    return spline



