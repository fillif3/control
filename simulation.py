from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from ExtendedKalman import ExtendedKalmanFilter

class Simulation:
    def __init__(self,func_system,func_system_derievative,func_observ,func_observ_derievative,controller,start_x
                 ,noise_system=0,noise_observation=0):
        self.func_system = func_system
        self.func_system_derievative = func_system_derievative
        self.func_observ = func_observ
        self.func_observ_derievative = func_observ_derievative
        self.cotroller=controller
        self.x=start_x
        self.x_history=[start_x]
        self.y = self.func_observ(self.x,noise_observation)
        self.y_history=[self.y]
        self.noise_observation=noise_observation
        self.noise_system=noise_system
        self.u_history=[[0]]
        self.t=[0]
        helper = np.concatenate([self.y,np.zeros([len(self.x)-len(self.y)])])
        kalman_func_system = lambda x,dt,u:Simulation.Runge_Kutta_next_step(self.func_system,x,dt,u,0)
        self.state_estimator=ExtendedKalmanFilter(kalman_func_system,func_system_derievative,func_observ,func_observ_derievative,
                                                  helper,noise_system,noise_observation)
        self.estimated_x =self.state_estimator.x
        self.estimated_x_history=[self.estimated_x]

    def simulate(self,dt,number_of_steps):
        for _ in range(number_of_steps):
            u = self.cotroller.next_input(self.y,self.t[-1])
            self.u_history.append(u)
            self.x=Simulation.Runge_Kutta_next_step(self.func_system,self.x,dt,u,self.noise_system)
            self.x_history.append(self.x)
            self.y = self.func_observ(self.x,self.noise_observation)
            self.estimate_state(u,dt)
            self.y_history.append(self.y)
            self.t.append(self.t[-1] + dt)

    @staticmethod
    def Runge_Kutta_next_step(func_system,x,dt,u,ns):
        k1 = dt*func_system(x,u)
        k2 = dt * func_system(x+k1/2, u)
        k3 = dt * func_system(x+k2/2, u)
        k4 = dt * func_system(x+k3/2, u)
        x=x+k1/6+k2/3+k3/3+k4/6
        x+=np.random.normal(0,ns)*dt
        return x


    def estimate_state(self,u,dt):
        self.state_estimator.update(self.y,u, dt)
        self.estimated_x=self.x
        self.estimated_x_history.append(self.estimated_x)

    def plot_estimation_error(self,indexes=None):
        if indexes is None:
            indexes = np.linspace(0,len(self.estimated_x_history[0])-1,len(self.estimated_x_history[0]))
        helper = np.abs(np.array(self.estimated_x_history)-np.array(self.x_history))
        for index in indexes:
            plt.plot(self.t,helper[:,int(index)],label="Estimation error: "+str(index+1))


    def plot_observations(self,indexes=None):
        if indexes is None:
            indexes = np.linspace(0,len(self.y_history[0])-1,len(self.y_history[0]))
        helper = np.array(self.y_history)
        for index in indexes:
            plt.plot(self.t,helper[:,int(index)],label="output: "+str(index+1))

    def plot_estimation(self,indexes=None):
        if indexes is None:
            print(self.estimated_x_history)
            indexes = np.linspace(0,len(self.estimated_x_history[0])-1,len(self.estimated_x_history[0]))
        print(self.estimated_x_history)
        helper = np.array(self.estimated_x_history)
        for index in indexes:
            plt.plot(self.t,helper[:,int(index)],label="Estimation: "+str(index+1))

    def plot_state(self,indexes=None):
        if indexes is None:
            indexes = np.linspace(0,len(self.x_history[0])-1,len(self.x_history[0]))
        helper = np.array(self.x_history)
        for index in indexes:
            plt.plot(self.t,helper[:,int(index)],label="Real state: "+str(index+1))

    def plot_input(self,indexes=None):
        if indexes is None:
            indexes = np.linspace(0,len(self.u_history[0])-1,len(self.u_history[0]))
        helper = np.array(self.u_history)
        for index in indexes:
            plt.plot(self.t,helper[:,int(index)],label="Real state: "+str(index+1))