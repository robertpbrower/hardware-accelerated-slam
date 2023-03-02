function [ekf_l, ekf_m, ekf_s] = ex4_slam()

    close all;
    
    % Process noise
    V = diag([0.02 0.5*pi/180].^2);
    % Sensing noise
    W = diag([0.1 1*pi/180].^2);
    % Initial mean
    x0 = [0 0 0]';
    % Initial covariance
    P0 = diag([.01 .01, 0.005].^2);

    load('e3.mat', 'odo_s', 'zind_s', 'z_s');

    

    [x_est, P_est, indices] = E3(odo_s, zind_s, z_s, V, W, x0, P0);

    visualize(x_est, P_est, indices);