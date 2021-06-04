import matplotlib.pyplot as plt
import numpy as np

from simulation import Simulation
from controllers import StepResponseController, P_controller,PI_controller, PID_controller
from systems import spring_system

NOISE_OBSERVATION=[0.1]
NOISE_SYSTEM=np.array([0.1,0.1])
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


# Press the green button in the gutter to run the script.


if __name__ == '__main__':


    for m in [1,10]:
        a, b, c, d = spring_system(1, 1, m)



        '''
        sys = signal.StateSpace(a, b, c, d)
        lti=sys.to_tf()
        print(lti)
    
        t, y = signal.step(lti)
    
        plt.plot(t, y)'''

        plt.xlabel('Time [s]')

        plt.ylabel('Amplitude')

        plt.grid()

        x=np.array([1,1])

        def func_system(x,u):

            return a.dot(x)+b.dot(u)

        def func_system_der(x,u):
            return a

        def func_obs(x,noise):
            return c.dot(x)+np.random.normal(0,noise)#np.array([x[0]+np.random.normal(0,noise)])

        def func_obs_der(x):
            return c

        #con =StepResponseController(1)
        con = PID_controller(1,lambda t: 1,1,1,1)

        sim = Simulation(func_system,func_system_der,func_obs,func_obs_der,con,x,NOISE_SYSTEM,NOISE_OBSERVATION)


        sim.simulate(0.02,2000)


        #sim.plot_observations()
        sim.plot_state()
        #sim.plot_estimation()



    plt.legend()
    plt.show()


    sim.plot_input()
    plt.show()
    #sim.plot_estimation_error()
    #plt.legend()
    #plt.show()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
