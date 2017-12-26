import matplotlib.pyplot as plt
import numpy as np

def viz():
    # Load the data
    data = np.loadtxt('/home/student/.ros/dbw.log', delimiter=',', skiprows=1)
    time       = data[:,0]
    throttle   = data[:,1]
    brake      = data[:,2]
    steering   = data[:,3]
    lx_target  = data[:,4]
    lx_current = data[:,5]
    az_target  = data[:,6]
    az_current = data[:,7]
    print(data.shape)
    plt.figure()
    plt.subplot(2,2,1)
    plt.plot(time, lx_target, 'b', label='lx_target')
    plt.plot(time, lx_current, 'r', label='lx_actual')
    plt.grid()
    plt.legend(loc='best')
    plt.xlabel('Time')
    plt.ylabel('Vel (m/s)')

    plt.subplot(2,2,2)
    plt.plot(time, az_target, 'b', label='az_target')
    plt.plot(time, az_current, 'r', label='az_actual')
    plt.ylim((-6.5, 6.5))
    plt.grid()    
    plt.legend(loc='best')
    plt.xlabel('Time')
    plt.ylabel('Ang-vel (r/s)')

    plt.subplot(2,2,3)
    plt.plot(time, throttle, 'g.', label='Throttle')
    plt.plot(time, brake/100, 'rs', label='Brake/100')
    plt.ylim((-1, 2))
    plt.legend(loc='best')    
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Throttle/Brake')

    plt.subplot(2,2,4)
    plt.plot(time, steering, 'r', label='Steer')
    plt.ylim((-3, 3))
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Steering')
    
    plt.show()


if __name__ == '__main__':
    viz()
