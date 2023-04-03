import numpy as np
import scipy
from E3 import E3
from visualize import visualize
    
def ex4_slam(): 
    # Process noise
    V = np.diag(np.array([0.02,0.5 * np.pi / 180]) ** 2)
    # Sensing noise
    W = np.diag(np.array([0.1,1 * np.pi / 180]) ** 2)
    # Initial mean
    x0 = np.array([[0],[0],[0]])
    # Initial covariance
    P0 = np.diag(np.array([0.01,0.01,0.005]) ** 2)
    input = scipy.io.loadmat("C:\\Users\\rober\\Documents\\HW_SW_Codesign\\project\\hardware-accelerated-slam\\e3.mat", simplify_cells = True)
    input2 = scipy.io.loadmat("C:\\Users\\rober\\Documents\\HW_SW_Codesign\\project\\hardware-accelerated-slam\\e3_new.mat", simplify_cells = True)

    odo_s = input['odo_s']
    zind_s = input['zind_s']
    z_s = input['z_s']
    map = input2['map']
    x_hist = input2['x_hist']
    (x_est,P_est,indices) = E3(odo_s,zind_s,z_s,V,W,x0,P0)
    visualize(map, x_hist, x_est,P_est)


if __name__=="__main__":
    ex4_slam()