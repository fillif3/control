import numpy as np

def spring_system(mass,friction,k):
    a=np.array([[0,1],[-k/mass,-friction/mass]])
    b=np.array([[0],[1/mass]])
    c=np.array([[1,0]])
    d=np.array([0])
    return a,b,c,d