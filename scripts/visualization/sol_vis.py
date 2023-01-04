from visualization.geo_vis import draw_obs
import rowan
from numpy import *
import numpy as np
import matplotlib.pyplot as plt

# line cyclers adapted to colourblind people
from cycler import cycler
line_cycler = (cycler(color=["#E69F00", "#56B4E9", "#009E73", "#0072B2", "#D55E00", "#CC79A7", "#F0E442"]) +
                 cycler(linestyle=["-", "--", "-.", ":", "-", "--", "-."]))
marker_cycler = (cycler(color=["#E69F00", "#56B4E9", "#009E73", "#0072B2", "#D55E00", "#CC79A7", "#F0E442"]) +
                 cycler(linestyle=["none", "none", "none", "none", "none", "none", "none"]) +
                 cycler(marker=["4", "2", "3", "1", "+", "x", "."]))

# adapted standard line cycler
standard_cycler = (cycler(color=["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#17becf", "#8c564b", "#e377c2", "#7f7f7f", "#9467bd", "#bcbd22"])+
				   cycler(linestyle=["-", "--", "-.", ":", "-", "--", "-.", "-", "--","-"]))

bw_cycler = cycler(color=["0","0.05","0.1","0.15","0.2","0.25","0.3","0.35","0.4","0.45","0.5"])

# set line cycler
plt.rc("axes", prop_cycle=standard_cycler)

# set latex font
plt.rcParams.update({
  "text.usetex": True,
  "font.family": "Computer Modern Roman",
})

SMALL_SIZE = 9#12
MEDIUM_SIZE = 10#15
BIGGER_SIZE = 10#18

plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=MEDIUM_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

def set_size(w,h, ax=None):
    """ w, h: width, height in inches """
    if not ax: ax=plt.gca()
    l = ax.figure.subplotpars.left
    r = ax.figure.subplotpars.right
    t = ax.figure.subplotpars.top
    b = ax.figure.subplotpars.bottom
    figw = float(w)/(r-l)
    figh = float(h)/(t-b)
    ax.figure.set_size_inches(figw, figh)


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def visualize_solution(robot,
					  data_scvx,
					  data_propagated_scvx,
					  obstacles,
					  int_err_scvx,
					  int_err_small_dt_scvx,
					  x0,xf,t_dil,xm= None,
					  nu = None):

	nTimeSteps_scvx = data_scvx.shape[0]
	timeVec_scvx = np.linspace(0, t_dil, nTimeSteps_scvx)

	###################################################################################################################
	fig, axs = plt.subplots(1, 1)

	# plot integration error Komo
	axs[0].plot(timeVec_scvx[1:], int_err_small_dt_scvx)
	axs[0].set_title("integration error SCVX (explicit euler with 10 times smaller step size)")
	axs[0].set_ylabel("int err small dt []")
	if robot.type == "dI":
		leg = ["$e_{I,p_x}$", "$e_{I,p_y}$", "$e_{I,p_z}$", "$e_{I,v_x}$", "$e_{I,v_y}$", "$e_{I,v_z}$"]
	# axs[2].legend(leg, loc="lower center", bbox_to_anchor=(0.5, -0.3))
	else:
		leg = ["$e_{I,p_x}$", "$e_{I,p_y}$", "$e_{I,p_z}$", "$e_{I,v_x}$", "$e_{I,v_y}$", "$e_{I,v_z}$",
			   "$e_{I,q_1}$", "$e_{I,q_2}$", "$e_{I,q_3}$", "$e_{I,q_4}$", "$e_{I,\omega_x}$", "$e_{I,\omega_y}$",
			   "$e_{I,\omega_z}$"]

	axs[0].legend(leg)
	axs[0].set_xlabel("time [s]")

	###################################################################################################################

	fig, axs = plt.subplots(2, 1)

	# plot int error
	axs[0].plot(timeVec_scvx[1:], int_err_scvx)
	axs[0].set_title("integration error")
	axs[0].set_ylabel("integration error []")
	if robot.type == "dI":
		leg = ["$e_{I,p_x}$", "$e_{I,p_y}$", "$e_{I,p_z}$", "$e_{I,v_x}$", "$e_{I,v_y}$", "$e_{I,v_z}$"]
	else:
		leg = ["$e_{I,p_x}$", "$e_{I,p_y}$", "$e_{I,p_z}$", "$e_{I,v_x}$", "$e_{I,v_y}$", "$e_{I,v_z}$",
					   "$e_{I,q_1}$", "$e_{I,q_2}$", "$e_{I,q_3}$", "$e_{I,q_4}$", "$e_{I,\omega_x}$","$e_{I,\omega_y}$",
					   "$e_{I,\omega_z}$"]

	axs[0].legend(leg)
	axs[0].set_xlabel("time [s]")

	# plot actions
	if robot.type == "dI":
		axs[1].plot(timeVec_scvx, data_scvx[:, 6:])
		axs[1].plot(timeVec_scvx, robot.max_u * np.ones(timeVec_scvx.shape), "-r")
		axs[1].legend(["$a_x$", "$a_y$", "$a_z$"])
		axs[1].set_ylabel("accelerations [m/s²]")
		inputCostSCVX = (data_scvx[:-1, 6:] ** 2).sum()
		axs[1].set_title("acceleration comparision \n input cost: \n SCVX: {}".format(np.round(inputCostSCVX, 3)))
	elif robot.type == "fM":
		axs[1].plot(timeVec_scvx, data_scvx[:, 13:])
		if robot.nrMotors == 2:
			leg = ["$f_1$", "$f_2$"]
		elif robot.nrMotors == 3:
			leg = ["$f_1$", "$f_2$", "$f_3$"]
		elif robot.nrMotors == 4:
			leg = ["$f_1$", "$f_2$", "$f_3$", "$f_4$"]
		elif robot.nrMotors == 6:
			leg = ["$f_1$", "$f_2$", "$f_3$", "$f_4$", "$f_5$", "$f_6$"]
		elif robot.nrMotors == 8:
			leg = ["$f_1$", "$f_2$", "$f_3$", "$f_4$", "$f_5$", "$f_6$", "$f_7$", "$f_8$"]

		axs[1].legend(leg)
		axs[1].set_ylabel("forces [N]")
		inputCostSCVX = (data_scvx[:-1, 13:] ** 2).sum()
		axs[1].set_title("force comparision \n input cost: {}".format(np.round(inputCostSCVX, 3)))
	axs[1].set_xlabel("time [s]")

	###################################################################################################################

	if robot.type == "dI":
		fig, axs = plt.subplots(1, 2)
		ax1 = axs[0]
		ax2 = axs[1]
	else:
		fig, axs = plt.subplots(2, 2)
		ax1 = axs[0,0]
		ax2 = axs[0,1]
		ax3 = axs[1,0]
		ax4 = axs[1,1]

	# plot positions
	ax1.plot(timeVec_scvx, data_scvx[:, :3])
	ax1.legend(["$p_x$", "$p_y$", "$p_z$"])
	ax1.set_xlabel("time [s]")
	ax1.set_ylabel("positions [m]")
	ax1.set_title("position comparision")

	# plot velocities
	ax2.plot(timeVec_scvx, data_scvx[:, 3:6])
	ax2.legend(["$v_x$", "$v_y$", "$v_z$"])
	ax2.set_xlabel("time [s]")
	ax2.set_ylabel("velocities [m/s]")
	ax2.set_title("velocity comparision")

	if robot.type == "fM":
		# plot quaternions
		ax3.plot(timeVec_scvx, data_scvx[:, 6:10])
		ax3.legend(["$q_1$", "$q_2$", "$q_3$", "$q_4$"])
		ax3.set_xlabel("time [s]")
		ax3.set_ylabel("quaternion [1]")
		ax3.set_title("quaternion comparision")

		# plot rotational velocities
		ax4.plot(timeVec_scvx, data_scvx[:, 10:13])
		ax4.legend(["$\omega_x$", "$\omega_y$", "$\omega_z$"])
		ax4.set_xlabel("time [s]")
		ax4.set_ylabel("rotational velocities [rad/s]")
		ax4.set_title("rotational velocities comparision")

	###################################################################################################################

	# plot the trajectory obtained by the optimizer
	fig = plt.figure()
	'''
	ax = plt.axes(projection='3d')
	ax.plot3D(data_scvx[:, 0], data_scvx[:, 1], data_scvx[:, 2], 'b', marker = 'o', markersize=4)
	ax.scatter3D(x0[0], x0[1], x0[2], color='k')
	ax.scatter3D(xf[0], xf[1], xf[2], color='k')
	if xm is not None:
		ax.scatter3D(xm[0], xm[1], xm[2], color='k')
	if obstacles is not None:
		for obs in obstacles:
			draw_obs(obs.type, obs.shape, obs.pos, ax)

	plt.legend(["SCVX"])

	ax = plt.gca()
	ax.set_xlim([robot.min_x[0], robot.max_x[0]])
	ax.set_ylim([robot.min_x[1], robot.max_x[1]])
	ax.set_zlim([robot.min_x[2], robot.max_x[2]])
	'''

	ax = fig.add_subplot(1, 1, 1, projection="3d")

	ax.plot(data_scvx[:, 0], data_scvx[:, 1], data_scvx[:, 2])

	if robot.type == "fM":
		for t in range(data_scvx.shape[1]):
			if t % 10 == 0:
				origin_scvx = data_scvx[t, :3]
				quat_scvx = data_scvx[t, 6:10]

				'''
				if "sphere" in prob_name:
					arrow_l = 0.4
					linewidth = 0.8
				else:'''

				arrow_l = 0.1
				linewidth = 0.6

				z_vec_scvx = np.vstack((origin_scvx, rowan.rotate(quat_scvx, np.array([0, 0, arrow_l]))))

				ax.quiver(z_vec_scvx[0, 0], z_vec_scvx[0, 1], z_vec_scvx[0, 2],
						  z_vec_scvx[1, 0], z_vec_scvx[1, 1], z_vec_scvx[1, 2], lw=linewidth, color="r")

	if obstacles is not None:
		for obs in obstacles:
			draw_obs(obs.type, obs.shape, obs.pos, ax)

	ax.set_ylabel("y [m]")
	ax.set_xlabel("x [m]")
	ax.set_zlabel("z [m]")
	set_axes_equal(ax)

	plt.show()
