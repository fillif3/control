import numpy as np

class LinearSystem:
    def __init__(self,a,b,c,d):
        self.A=a
        self.B=b
        self.C=c
        self.D=d

    def func_system(self,x, u):
        return self.A.dot(x) + self.B.dot(u)

    def func_system_der(self,x, u):
        return self.A

    def func_obs(self,x, noise):
        return self.C.dot(x) + np.random.normal(0, noise)  # np.array([x[0]+np.random.normal(0,noise)])

    def func_obs_der(self,x):
        return self.C

class Spring(LinearSystem):
    def __init__(self, mass, friction, k):
        super().__init__(np.array([[0,1],[-k/mass,-friction/mass]]), np.array([[0],[1/mass]]), np.array([[1,0]]), np.array([0]))


