import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

from systems import  spring_system

a, b, c, d = spring_system(1, 1, 1)


sys = signal.StateSpace(a, b, c, d)
lti=sys.to_tf()
print(lti)
print(lti.poles)