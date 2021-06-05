import matplotlib.pyplot as plt
import numpy as np

from simulation import Simulation
from controllers import StepResponseController,  PID_controller
from systems import Spring

NOISE_OBSERVATION=np.array([0.1])
NOISE_SYSTEM=np.array([1,100])
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


# Press the green button in the gutter to run the script.


if __name__ == '__main__':

    costMin=1000000
    P_min=None
    I_min=None
    D_min=None
    #for p in [0,0.5,1,2.5,5,10]:
    #    for i in [0,0.5, 1, 2.5, 5, 10]:
    #        for d in [0, 0.5, 1, 2.5, 5, 10]:
    system= Spring(1, 1, 1)

    plt.xlabel('Time [s]')

    plt.ylabel('Amplitude')

    plt.grid()

    x=np.array([1,1])

    #con =StepResponseController(1)
    ref = lambda t:np.array( [np.sin(1*t)])
    #ref2=lambda t:1
    con = PID_controller(1,ref,30,2.5,5)

    sim = Simulation(system,con,x,NOISE_SYSTEM,NOISE_OBSERVATION)


    sim.simulate(0.02,800)

    cost=sim.costFunction(np.array([1]), np.array([0]), ref, [0])


    #sim.plot_observations()
    sim.plot_state()
    arr=[]
    #sim.plot_estimation()
    for t in sim.t:
        arr.append(ref(t))
    plt.plot(sim.t,arr)



    print('final')
    print(P_min)
    print(I_min)
    print(D_min)
    plt.legend()
    plt.show()


    sim.plot_input()
    plt.show()
    #sim.plot_estimation_error()
    #plt.legend()
    #plt.show()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
