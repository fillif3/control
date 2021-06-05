import numpy as np



class StepResponseController:
    def __init__(self,dimension):
        self.dimension=dimension

    def next_input(self,y,t):
        k=np.array([1]*self.dimension)
        return k

class PID_controller:
    def __init__(self,dimension,ref_func,p,i,d):
        self.ref_func=ref_func
        self.dimension=dimension
        self.kp=p
        self.ki=i
        self.kd=d
        self.previous_t=0
        self.integrator=0
        self.previous_error=0

    def next_input(self,y,t):
        dt = t- self.previous_t
        self.previous_t=t
        r = self.ref_func(t)
        error = r-y
        self.integrator+=error*dt
        if dt>0:
            derievativ= (error-self.previous_error)/dt
        else: derievativ=0
        self.previous_error=error
        return self.kp*error+self.ki*self.integrator+self.kd*derievativ

class MPC:
    def __init__(self,controlot_horizon,observation_horizon,system):
        pass
