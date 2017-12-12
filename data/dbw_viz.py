import matplotlib.pyplot as plt
import numpy as np

def viz():
    # Load the data
    data = np.loadtxt('dbw.log', delimiter=',', skiprows=1)
    print(data.shape)
    plt.figure()
    plt.subplot(2,2,1)
    plt.plot(data[:,0], data[:,1], 'b', label='lx_target')
    plt.plot(data[:,0], np.abs(data[:,2]), 'r', label='lx_actual')
    plt.grid()
    plt.legend(loc='best')
    plt.xlabel('Time')
    plt.ylabel('Vel (m/s)')

    plt.subplot(2,2,2)
    plt.plot(data[:,0], data[:,3], 'b', label='az_target')
    plt.plot(data[:,0], data[:,4], 'r', label='az_actual')
    plt.ylim((-6.5, 6.5))
    plt.grid()    
    plt.legend(loc='best')
    plt.xlabel('Time')
    plt.ylabel('Ang-vel (r/s)')

    plt.subplot(2,2,3)
    plt.plot(data[:,0], data[:,5], 'g.', label='Throttle')
    plt.plot(data[:,0], data[:,6]/100, 'rs', label='Brake/100')
    plt.ylim((-1, 2))
    plt.legend(loc='best')    
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Throttle/Brake')

    plt.subplot(2,2,4)
    plt.plot(data[:,0], data[:,7], 'r', label='Steer')
    plt.ylim((-3, 3))
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Steering')
    
    plt.show()


if __name__ == '__main__':
    viz()
