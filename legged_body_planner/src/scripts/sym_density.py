#! /usr/bin/env python3

# write a class with one method
# method should take a list of parameters and return a value

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
from sympy.vector import CoordSys3D
import math


class Density:
    def __init__(self, r1=1, r2=2, obs_center=[0, 0], goal=[5, 5], alpha=0.2, gain=100, saturation=2, rad_from_goal=0.25):
        """
        Inputs: 
        r1              : radius of the obstacle
        r2              : sensing radius for the obstalce
        obs_center      : list of (x,y) position the center of the obstacle
        goal            : (x,y) position of the goal
        alpha           : tuning parameter for the density function
        gain            : Control multiplier
        saturation      : Saturation level for control
        """
        self.r1 = r1
        self.r2 = r2
        self.alpha = alpha
        self.obs_center = obs_center
        self.goal = goal
        self.gain = gain
        self.saturation = saturation
        self.terminated = False
        self.rad_from_goal = rad_from_goal

    def distance_metric(self):
        """
        Form the distance function V
        Outputs:
        dist            : the scalar distance metric for the robot
        """
        x, y = sp.symbols('x y')
        R = CoordSys3D('R')
        x_vec = x*R.i + y*R.j
        goal = self.goal[0]*R.i + self.goal[1]*R.j
        dist = sp.sqrt((x_vec-goal).dot(x_vec-goal))
        dist_fn = sp.lambdify([x, y], 1/(dist**(2*self.alpha)))
        return dist, dist_fn

    def inverse_bump(self):
        """
        Form the bump function as psi
        Outputs:
        ivnerse_bump    : the circular inverse bump function evaltuated at x 
        """
        x, y = sp.symbols('x y')
        R = CoordSys3D('R')
        x_vec = x*R.i + y*R.j
        obs = self.obs_center[0]*R.i + self.obs_center[1]*R.j
        obs_dist = sp.sqrt((x_vec-obs).dot(x_vec-obs))
        shape = (np.subtract(obs_dist**2, self.r1**2)) / \
            np.subtract(self.r2**2, self.r1**2)

        f = sp.Piecewise((0, shape <= 0), (sp.exp(-1/shape), shape > 0))
        shape_shift = 1-shape
        f_shift = sp.Piecewise((0, shape_shift <= 0),
                               (sp.exp(-1/shape_shift), shape_shift > 0))
        inverse_bump = f/(f+f_shift)
        inverse_bump_fn = sp.lambdify([x, y], inverse_bump)
        return inverse_bump, inverse_bump_fn

    def grad_inverse_bump(self):
        """
        Form the gradient of the inverse bump function
        """
        x, y = sp.symbols('x y')
        inverse_bump, inverse_bump_fn = self.inverse_bump()
        grad_psi_x = sp.diff(inverse_bump, x)
        grad_psi_y = sp.diff(inverse_bump, y)
        grad_psi_fn_x = sp.lambdify([x, y], grad_psi_x)
        grad_psi_fn_y = sp.lambdify([x, y], grad_psi_y)
        return grad_psi_fn_x, grad_psi_fn_y

    def density(self):
        """
        Form the density function as rho= (1/V^(2*alpha))*psi
        Inputs:
        x               : the (x,y) position of the robot
        Outputs:
        rho             : the density function evaltuated at x 
        """
        x, y = sp.symbols('x y')
        R = CoordSys3D('R')
        x_vec = x*R.i + y*R.j

        # distance function
        goal = self.goal[0]*R.i + self.goal[1]*R.j
        dist = sp.sqrt((x_vec-goal).dot(x_vec-goal))

        # inverse bump function
        obs = self.obs_center[0]*R.i + self.obs_center[1]*R.j
        obs_dist = sp.sqrt((x_vec-obs).dot(x_vec-obs))
        shape = (np.subtract(obs_dist**2, self.r1**2)) / \
            np.subtract(self.r2**2, self.r1**2)
        f = sp.Piecewise((0, shape <= 0), (sp.exp(-1/shape), shape > 0))
        shape_shift = 1-shape
        f_shift = sp.Piecewise((0, shape_shift <= 0),
                               (sp.exp(-1/shape_shift), shape_shift > 0))
        inverse_bump = f/(f+f_shift)

        # density function
        rho = 1/(dist**(2*self.alpha))*inverse_bump
        rho_fn = sp.lambdify([x, y], rho)

        return rho, rho_fn

    def grad_density(self):
        """
        Form the gradient of the density function
        """
        x, y = sp.symbols('x y')
        rho, rho_fn = self.density()
        grad_rho_x = sp.diff(rho, x)
        grad_rho_y = sp.diff(rho, y)
        grad_rho_fn_x = sp.lambdify([x, y], grad_rho_x)
        grad_rho_fn_y = sp.lambdify([x, y], grad_rho_y)
        return grad_rho_fn_x, grad_rho_fn_y

    def eval_distance_fn(self, x_domain=np.linspace(-10, 10, 100), y_domain=np.linspace(-10, 10, 100)):
        """
        Evalulate the inverse bump function
        Inputs:
        x_domain        : the x domain to evaluate
        y_domain        : the y domain to evaluate
        Outputs:
        f_inverse_bump  : the value of inverse bump function at each point in the domain
        """
        f_distance = []
        dist, distance_fn = self.distance_metric()
        for x in x_domain:
            for y in y_domain:
                f_distance.append(distance_fn(x, y))
        return f_distance

    def eval_inverse_bump(self, x_domain=np.linspace(-10, 10, 100), y_domain=np.linspace(-10, 10, 100)):
        """
        Evalulate the inverse bump function
        Inputs:
        x_domain        : the x domain to evaluate
        y_domain        : the y domain to evaluate
        Outputs:
        f_inverse_bump  : the value of inverse bump function at each point in the domain
        """
        f_inverse_bump = []
        inverse_bump, inverse_bump_fn = self.inverse_bump()
        for x in x_domain:
            for y in y_domain:
                f_inverse_bump.append(inverse_bump_fn(x, y))
        return f_inverse_bump

    def eval_density(self, x_domain=np.linspace(-10, 10, 100), y_domain=np.linspace(-10, 10, 100)):
        """
        Evalulate the density function
        Inputs:
        x_domain        : the x domain to evaluate
        y_domain        : the y domain to evaluate
        Outputs:
        f_density       : the value of density function at each point in the domain
        """
        f_density = []
        rho, rho_fn = self.density()
        for x in x_domain:
            for y in y_domain:
                if rho_fn(x, y) < 10:
                    f_density.append(rho_fn(x, y))
                else:
                    f_density.append(10)
        return f_density

    def eval_grad_inverse_bump(self, x_domain=np.linspace(-10, 10, 100), y_domain=np.linspace(-10, 10, 100)):
        """
        Evalulate the density function
        Inputs:
        x_domain        : the x domain to evaluate
        y_domain        : the y domain to evaluate
        Outputs:
        f_grad_density  : the value of gradient of density function at each point in the domain
        """
        # f_grad_psi_x = []
        # f_grad_psi_y = []
        # for x in x_domain:
        #     for y in y_domain:
        #             grad_psi_x, grad_psi_y = self.grad_inverse_bump([x,y])
        #             f_grad_psi_x.append(grad_psi_x)
        #             f_grad_psi_y.append(grad_psi_y)
        # return f_grad_psi_x, f_grad_psi_y

    def eval_grad_density(self, x_domain=np.linspace(-10, 10, 100), y_domain=np.linspace(-10, 10, 100)):
        """
        Evalulate the density function
        Inputs:
        x_domain        : the x domain to evaluate
        y_domain        : the y domain to evaluate
        Outputs:
        f_grad_density  : the value of gradient of density function at each point in the domain
        """
        # f_grad_density_x = []
        # f_grad_density_y = []
        # for x in x_domain:
        #     for y in y_domain:
        #             grad_density_x, grad_density_y = self.grad_density([x,y])
        #             f_grad_density_x.append(grad_density_x)
        #             f_grad_density_y.append(grad_density_y)
        # return f_grad_density_x, f_grad_density_y

    def get_plan(self, current_t, x0, y0, N, dt):
        # forawrd euler
        grad_rho_fn_x, grad_rho_fn_y = self.grad_density()
        rad_from_goal = self.rad_from_goal
        saturation = self.saturation
        # gain = 80
        gain = self.gain

        x = np.zeros((2, N))
        u = np.zeros((2, N))
        t = np.zeros((1, N))
        x[0] = x0
        x[1] = y0
        t[0, 0] = current_t
        for i in range(1, N):
            try:
                u[0, i-1] = gain*grad_rho_fn_x(x[0, i-1], x[1, i-1])
                u[1, i-1] = gain*grad_rho_fn_y(x[0, i-1], x[1, i-1])
            except RuntimeWarning:
                # Evaluated at target point, not in formulation
                # Turn to zero
                print("Evaluated at target set... Setting control to 0")
                u[0, i-1] = 0
                u[1, i-1] = 0

            # check if goal is reached
            dist = np.linalg.norm(np.subtract(x[:, i-1], self.goal))
            if dist <= rad_from_goal:
                # # LQR Controller
                # K = 0.3162  # LQR gain for single integrator
                # u[0, i-1] = - K*x[0, i-1]
                # u[1, i-1] = - K*x[1, i-1]

                # """
                # print("rad_from_goal - 0.1 > 0: %d" % (rad_from_goal-0.1 > 0))
                # print("Dist: ", dist)
                # print("rad_from_goal - 0.1: %0.2E " % (rad_from_goal - 0.1))
                # print("dist < rad_from_goal - 0.1: %0.2f" %
                #       (dist < rad_from_goal - 0.1))
                # """
                # if (rad_from_goal - 0.1 >= 0 and dist <= rad_from_goal - 0.1):
                #     print("Second layer")
                #     u[0, i-1] = 0
                #     u[1, i-1] = 0

                # Zero Controller
                u[0, i-1] = 0
                u[1, i-1] = 0

            # saturate the control inputs
            if np.max(u[:, i-1]) >= saturation:
               # print('saturation')
                u[:, i-1] = (u[:, i-1]/np.max(u[:, i-1]))*saturation

            # propagate the states
            x[0, i] = x[0, i-1] + dt*u[0, i-1]
            x[1, i] = x[1, i-1] + dt*u[1, i-1]
            t[0, i] = t[0, i-1] + dt
        # add value of u[N] = u[N-1] since we dont compute u[N]
        # need trajectory length N to be the same for both u and x
        u[:, -1] = u[:, -2]
        return t, x, u

    # Utility functions w/in class

    def getYaw(self, curr_vel, prev_yaw=None):
        """
        Calculates heading angle given tangent (vel) vectors

        Inputs:
        -------
        curr_vel : Current velocity (x_dot, y_dot)
        prev_yaw : Previous yaw angle

        Output:
        --------
        yaw : float
            Current yaw angle
        """
        curr_vel_x = curr_vel[0]
        curr_vel_y = curr_vel[1]
        if prev_yaw == None:
            # print("yaw ref: ", math.atan2(curr_vel_y, curr_vel_x))
            return math.atan2(curr_vel_y, curr_vel_x)  # Assumes 2d
        else:
            wrapped_yaw = math.atan2(curr_vel_y, curr_vel_x)
            # Assumes yaw rate is 'slow'
            # print("Diff: ", wrapped_yaw - prev_yaw)

            if (wrapped_yaw - prev_yaw) > math.pi:
                quotient = int((wrapped_yaw - prev_yaw)/(2*math.pi))
                wrapped_yaw = wrapped_yaw - math.pi - quotient*(2*math.pi)
            elif (wrapped_yaw - prev_yaw) < -math.pi:
                quotient = int((wrapped_yaw - prev_yaw)/(2*math.pi))
                wrapped_yaw = wrapped_yaw + math.pi - quotient*(2*math.pi)
        # print("yaw ref: ", math.atan2(curr_vel_y, curr_vel_x))
        return wrapped_yaw

########### utility functions ###########################################################


def symlog(x):
    """ Returns the symmetric log10 value """
    return np.sign(x) * np.log10(np.abs(x))


###################### main function ####################################################
def main():
    plot_density = False
    plot_traj = True
    density = Density()

    N = 5000
    dt = 0.01
    x0 = -2
    y0 = -3
    current_t = 0
    rad_from_goal = 0.1
    t, x, u = density.get_plan(current_t, x0, y0, N, dt)

    ############################ plots for verificaion ######################################################
    if (plot_density == True):
        # surface plot of f_density
        X = np.linspace(-10, 10, 100)
        Y = np.linspace(-10, 10, 100)
        f_distance = density.eval_density(X, Y)

        fig = plt.figure()
        X, Y = np.meshgrid(X, Y)
        ax = fig.add_subplot(2, 2, 1, projection='3d')
        Z = np.array(f_distance).reshape(100, 100)
        surf1 = ax.plot_surface(X, Y, Z, cmap=plt.cm.coolwarm,
                                linewidth=0, antialiased=False)
        ax.zaxis.set_major_locator(plt.LinearLocator(10))
        ax.zaxis.set_major_formatter(plt.FormatStrFormatter('%.02f'))
        fig.colorbar(surf1, shrink=0.5, aspect=5)

    if (plot_traj == True):
        fig = plt.figure()
        ax = fig.add_subplot(2, 2, 2)
        ax.scatter(x[0, :-2], x[1, :-2])
        # ax.plot(t, x[0, :-2])
        # ax.plot(t, x[1,:-2])
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        plt.show()


if __name__ == "__main__":
    main()
