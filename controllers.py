import numpy as np

class StepResponseController:
    def __init__(self,dimension):
        self.dimension=dimension

    def next_input(self,y,t):
        k=np.array([1]*self.dimension)
        return k

class P_controller:
    def __init__(self,dimension,ref_func,p):
        self.ref_func=ref_func
        self.dimension=dimension
        self.kp=p

    def next_input(self,y,t):
        r = self.ref_func(t)
        error = r-y
        return self.kp*error

class PI_controller:
    def __init__(self,dimension,ref_func,p,i):
        self.ref_func=ref_func
        self.dimension=dimension
        self.kp=p
        self.ki=i
        self.previous_t=0
        self.integrator=0

    def next_input(self,y,t):
        dt = t- self.previous_t
        self.previous_t=t
        r = self.ref_func(t)
        error = r-y
        self.integrator+=error*dt
        return self.kp*error+self.ki*self.integrator

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