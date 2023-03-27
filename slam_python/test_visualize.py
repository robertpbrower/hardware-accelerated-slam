import scipy
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

from visualize import visualize


output = scipy.io.loadmat("C:\\Users\\rober\\Documents\\HW_SW_Codesign\\project\\hardware-accelerated-slam\\sample_out.mat", simplify_cells = True)
input = scipy.io.loadmat("C:\\Users\\rober\\Documents\\HW_SW_Codesign\\project\\hardware-accelerated-slam\\e3_new.mat", simplify_cells = True)

map = input['map']
x_hist = input['x_hist']
x_est = output['x_est']
p_est = output['P_est']


visualize(map, x_hist, x_est, p_est)