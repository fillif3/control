from scipy import signal
import matplotlib.pyplot as plt
from copy import deepcopy
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self,func_system,func_system_derievative,func_observ,func_observ_derievative,starting_vactor,noise_system,noise_observation):
        self.func_system=func_system
        self.func_system_derievative=func_system_derievative
        self.func_observ=func_observ
        self.func_observ_derievative=func_observ_derievative
        self.x=starting_vactor
        self.R=np.diag(noise_system)
        helper = np.concatenate([noise_observation, np.array((len(self.x) - len(noise_observation))*[100000])])
        self.Q=np.diag(noise_observation)
        self.P = np.diag(helper)

    def update(self,observation,u,dt):
        x_prediction = self.func_system(self.x,dt,u)
        F =self.func_system_derievative(self.x,dt)
        P_prediction = F.dot(self.P).dot(F.T)+self.R
        error = observation-self.func_observ(x_prediction,0)
        H = self.func_observ_derievative(x_prediction)
        S=H.dot(P_prediction).dot(H.T)+self.Q
        K=P_prediction.dot(H.T).dot(np.linalg.inv(S))
        self.x = x_prediction+K.dot(error)
        self.P=(np.eye(len(self.x))-K.dot(H)).dot(P_prediction)

if __name__ == "__main__":
    a=np.array([1,3])
    b=np.array([1,8])
    print(np.multiply(a,b))
