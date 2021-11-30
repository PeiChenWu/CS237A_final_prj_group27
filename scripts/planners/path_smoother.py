import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    dist_between_points = [np.linalg.norm(np.array(path[i+1])-np.array(path[i])) for i in range(len(path)-1)]
    
    nominal_time = [int(np.ceil(dist/V_des)) for dist in dist_between_points]
    
    total_time = np.sum(nominal_time)
    t_smoothed = np.linspace(0, total_time, int(total_time/dt))
    
    time_pts = np.array([np.sum(nominal_time[:i]) for i in range(len(nominal_time)+1)])
    
    x_pts = np.array([loc[0] for loc in path])
    y_pts = np.array([loc[1] for loc in path])
    
    spl_x = scipy.interpolate.splrep(time_pts, x_pts, s=alpha)
    spl_y = scipy.interpolate.splrep(time_pts, y_pts, s=alpha)
    
    x_d = scipy.interpolate.splev(t_smoothed, spl_x)
    y_d = scipy.interpolate.splev(t_smoothed, spl_y)
    
    xd_d = scipy.interpolate.splev(t_smoothed, spl_x, der=1)
    yd_d = scipy.interpolate.splev(t_smoothed, spl_y, der=1)

    xdd_d = scipy.interpolate.splev(t_smoothed, spl_x, der=2)
    ydd_d = scipy.interpolate.splev(t_smoothed, spl_y, der=2)
    
    theta_d = np.arctan2(yd_d,xd_d)
    
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
