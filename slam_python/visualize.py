

import scipy
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np



def visualize(map, x_hist, x_est, p_est):
    T = len(x_est) - 1
    M = int((len(x_est[T]) - 3) / 2)
    m_est = np.reshape(x_est[T][3:], tuple(np.array([2,M])), order="F")

    ells = [Ellipse(xy=(m_est[0,i], m_est[1,i]),
                    width=np.sqrt(p_est[T][2*(i+1) + 1, 2*(i+1) + 1] * 5.991*10),
                    height=np.sqrt(p_est[T][2*(i+1) + 2, 2*(i+1) + 2] * 5.991*10))   
                        for i in range(M)]

    ells.extend(
    [Ellipse(xy=(x_est[i][0], x_est[i][1]),
                    width=np.sqrt(p_est[i][0, 0] * 5.991*10),
                    height=np.sqrt(p_est[i][1,1] * 5.991*10))   
                        for i in range(0,1000,10)] 
    )

    fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
    for e in ells:
        ax.add_artist(e)
        e.set_fill(False)
        e.set_edgecolor('r')


    plt.plot([x_hist[i][0] for i in range(1000)], [x_hist[i][1] for i in range(1000)] ,'b')
    plt.scatter(map[0], map[1], c='b', s=20, marker='*')
    plt.plot([x_est[i][0] for i in range(1000)], [x_est[i][1] for i in range(1000)] ,'r')
    plt.scatter(m_est[0], m_est[1], c='r', s=10)

    plt.show()