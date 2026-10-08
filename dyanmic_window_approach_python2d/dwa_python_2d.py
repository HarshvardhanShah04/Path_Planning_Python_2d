from matplotlib import pyplot as plt
from scipy import interpolate
import random
import math
import numpy as np

obstacles_on = 1;

class parameters:
    def __init__(self):

        self.goal = np.array([4,4]);
        self.obstacles = np.array([
        [1,1.2,0.9],
        # [3.5,1,0.8],
        [5,2.2,0.8],
        [-2.8,-2.2,0.6],
        [3.3,3,0.4],
        [0.8,-1.7,0.8],
        [-3.6,1.8,3],
        [3,-4,1],
        # [2,3,0.7]
        ]); #x,y,R
        

        self.r0 = 0.1; #when the robot is at this distance from goal then stop
        self.R = 0.1; #radius of the robot

        self.safety_margin = 1*self.R

        self.accel = 0.2;
        self.v_max = 0.5; #max speed
        self.v_min = 0.08;

        self.v_pref = 0.2; #nominal speed
        self.max_vel_devtn = max(abs(self.v_min - self.v_pref), abs(self.v_max - self.v_pref))

        self.alpha = 0.5;
        self.omega_min = -1.0;
        self.omega_max = 1.0;

        self.n_omega = 10; #number of omega's to generate
        self.n_v = 10


        self.prediction_horizon = 30; #simulate this much time ahea ahead
        self.dt = 0.1; #integration step

        self.pause = 0.001
        self.fps = 10

def dwa(x0,y0,theta0,v_curr,omega_curr,parms):

    n_omega = parms.n_omega
    n_v = parms.n_v

    accel = parms.accel
    v_max = parms.v_max
    v_min = parms.v_min

    v_pref = parms.v_pref

    alpha = parms.alpha
    omega_min = parms.omega_min
    omega_max = parms.omega_max

    goal = parms.goal

    dt = parms.dt
    prediction_horizon = parms.prediction_horizon

    obstacles = parms.obstacles
    obs_inflation = parms.R + parms.safety_margin

    max_vel_deviation = parms.max_vel_devtn



    #1) generate n different omegas
    v_low = max(v_min, v_curr - accel*dt)
    v_high = min(v_max, v_curr + accel*dt)

    omega_low =  max(omega_min, omega_curr - alpha*dt)
    omega_high = min(omega_max, omega_curr + alpha*dt)

    omega_sample = np.linspace(omega_low,omega_high,n_omega);
    v_sample = np.linspace(v_low, v_high, n_v);

    wg = 2;
    wh = 1.5;
    wo = 5;
    wv = 1.2;
    w_sum = wg+wh+wo+wv

    #2) generate n trajectories for various omega and v
    #3) generate n cost function for various trajectories
    overall_cost_all = np.zeros((n_v, n_omega));

    max_dist = v_max*prediction_horizon*dt
    
    for i in range(n_v):
        for j in range(n_omega):
            z0 = [x0, y0, theta0]
            z = np.array([z0]);
            for h in range(0,prediction_horizon):
                z0 = euler_integration([0, dt],z0,[v_sample[i],omega_sample[j]],parms)
                z = np.vstack([z, z0])

            #cost to goal
            goal_cost_norm = goal_cost_calc(goal, z[0,:], z[-1,:], (max_dist))

            #cost of obstacle
            obst_cost_norm = obst_cost_calc(obstacles, z, obs_inflation, parms.R)
            
            #cost heading
            head_cost_norm = cost_heading(goal, z[-1,:])/math.pi

            #cost velocity
            vel_cost_norm = abs(v_sample[i] - v_pref)/max_vel_deviation 

            overall_cost_all[i,j] = (wg*goal_cost_norm + wo*obst_cost_norm + wh*head_cost_norm + wv*vel_cost_norm)/(w_sum)


    #4) choose omega with the lowest cost
    min_cost = math.inf
    omega_opt = 0
    v_opt = 0

    for i in range(0,n_v):
        for j in range(0,n_omega):
            if (overall_cost_all[i,j]<min_cost):
                min_cost = overall_cost_all[i,j];
                omega_opt = omega_sample[j];
                v_opt = v_sample[i];

    print("Minimum cost:", np.min(overall_cost_all))
    print("Finite trajectories:", np.sum(np.isfinite(overall_cost_all)))
    print("Selected velocity:", v_opt)
    print("Selected omega:", omega_opt)

    return [v_opt,omega_opt]


def goal_cost_calc(goal_pos,bot_pose_init, bot_pose_final, max_dist):
    df = math.sqrt((goal_pos[0] - bot_pose_final[0])**2 + (goal_pos[1] - bot_pose_final[1])**2)
    di = math.sqrt((goal_pos[0] - bot_pose_init[0])**2 + (goal_pos[1] - bot_pose_init[1])**2)

    jg = (di - df)

    if (max_dist>di):
        return (1 - jg/max_dist)
    else:
        return (df/di)


def obst_cost_calc(obst, traj, inflat_rad, bot_rad):
    min_clearance = math.inf
    n_traj = len(traj)
    n_obs = len(obst)

    for i in range(n_traj):
        for o in range(n_obs):
            centre_distance = math.sqrt((traj[i,0] - obst[o,0])**2 + (traj[i,1] - obst[o,1])**2)
            clearance = (centre_distance - obst[o,2] - bot_rad)
            if (clearance < min_clearance): min_clearance = clearance

    influence_dist = 10*bot_rad
    if (min_clearance <= inflat_rad):
        return math.inf
    elif (min_clearance < influence_dist):
        # return ((influence_dist - min_clearance) / (influence_dist - inflat_rad))**2
        return(10/(influence_dist**2))*(min_clearance - influence_dist)**2
    else:
        return 0


def cost_heading(goal_pos, bot_pose_final):
    th_des_f = math.atan2((goal_pos[1] - bot_pose_final[1]), goal_pos[0] - bot_pose_final[0])
    error_f = (th_des_f - bot_pose_final[2])

    error_f = math.atan2(math.sin(error_f), math.cos(error_f))

    return abs(error_f)

def animate(t, z, parms):

    R = parms.R
    phi = np.arange(0, 2*np.pi, 0.1)

    x_goal = parms.goal[0]
    y_goal = parms.goal[1]

    fig, ax = plt.subplots()

    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')

    if obstacles_on == 1:
        for obs in parms.obstacles:
            x_center = obs[0]
            y_center = obs[1]
            r_obs = obs[2]

            x_obs = x_center + r_obs * np.cos(phi)
            y_obs = y_center + r_obs * np.sin(phi)

            ax.plot(x_obs, y_obs, color='red')

    ax.plot(
        x_goal,
        y_goal,
        'ko',
        markersize=10,
        markerfacecolor='black'
    )

    robot, = ax.plot([], [], color='black')
    heading, = ax.plot([], [], color='black')

    # Path
    path, = ax.plot([], [], color='blue')

    for i in range(len(t)):

        x = z[i, 0]
        y = z[i, 1]
        theta = z[i, 2]

        # Robot circle
        x_robot = x + R * np.cos(phi)
        y_robot = y + R * np.sin(phi)

        robot.set_data(x_robot, y_robot)

        # Robot heading
        x2 = x + R * np.cos(theta)
        y2 = y + R * np.sin(theta)

        heading.set_data([x, x2], [y, y2])

        # Path
        path.set_data(z[:i+1, 0], z[:i+1, 1])

        plt.pause(0.001)

    plt.show()


def euler_integration(tspan,z0,u,parms):
    v = u[0]

    v_max = parms.v_max;
    if (v>=v_max):
        v = v_max;

    omega = u[1]
    dh = tspan[1]-tspan[0]

    x0 = z0[0]
    y0 = z0[1]
    theta0 = z0[2]

    xdot_c = v*math.cos(theta0)
    ydot_c = v*math.sin(theta0)
    thetadot = omega

    x1 = x0 + xdot_c*dh
    y1 = y0 + ydot_c*dh
    theta1 = theta0 + thetadot*dh

    z1 = [x1, y1, theta1]
    return z1



parms = parameters()
direction = random.choice([-1, 1]) #randomly generate a direction to turn in the beginnig

#initial condition, [x0, y0, theta0]
z0 = [-4.9, -4.9, 0]

N = 1500 #end time
h = parms.dt
x_goal = parms.goal[0]
y_goal = parms.goal[1]
r0 = parms.r0
# %%%%% the controls are v = speed and omega = direction
# %%%%%% v = 0.5r(phidot_r + phidot_l)
# %%%%%% omega = 0.5 (r/b)*(phitdot_r - phidot_l)
z = np.array([z0]);
t = np.array([0]);

omega_current = 0;
v_current = 0;

for i in range(0,N):

    x = z0[0]; y = z0[1]; theta = z0[2];
    theta_des = np.arctan2( y_goal - y,x_goal - x);

    
    values = dwa(x,y,theta,v_current, omega_current ,parms);
    v_current = values[0]
    omega_current = values[1]

    r = np.sqrt((x_goal - x)**2 + (y_goal - y)**2)
    if (r < r0):
        v_current = 0 #come to stop
        break; #exit for loop

    z0 = euler_integration([0, h],z0,[v_current,omega_current],parms)
    z = np.vstack([z, z0])
    t = np.append(t,t[-1]+h)

animate(t,z,parms)
plt.show(block = True)
