# Visualizes the estimated trajectory and landmarks, ground truth,
# and optional toolbox EKF estimate (computed only if sol passed in).

# Input: x_est -> 1xT cell array containing the map state mean
#                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
#                 where M is the number of landmarks observed by time t)
#        P_est -> 1xT cell array containing the vehicle state covariance
#                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
#                 where M is the number of landmarks observed by time t)
#        indices -> Mx1 vector containing the landmark index corresponding
#                   to the entries in the state vector, where M is the
#                   number of landmarks observed by the final time step T)
#                   For example, if indices is [15; 4], then the first
#                   three rows of x_est and P_est correspond to the
#                   vehicle state, the next two correspond to landmark 15,
#                   and the next two rows correspond to landmark 4, etc.
#        map -> Robotics toolbox Map object containing the
#               known map (known landmarks)
#        veh -> Robotics toolbox Vehicle object containing the
#               known vehicle (known trajectory)
#        mode -> 'l' for localization, 'm' for mapping, 's' for SLAM,
#                'h' for statistics
#                In 'l', 'm', 's' modes, the ground truth and estimated
#                trajectory and/or map is visualized, and estimation errors
#                are printed out if compute_errors is true
#                In 'h' mode, visualization and printing is suppressed,
#                since many trials are assumed.
#        sol -> Robotics toolbox EKF object containing the toolbox estimate;
#               if passed in, the estimate will be visualized and errors
#               will be computed; if no solution, pass in empty matrix []
#        compute_errors -> Boolean value; if true, errors are computed;
#                          if false, estimation errors are not computed
#                          (unless in 'h' mode); this allows partial
#                          trajectories and maps to be visualized as well.
# Output: statistics -> 1x12 vector of error statistics
#         = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe,
#            err_xtm, err_etm, err_xem, num_xt, num_et, num_xe]
#         There are 4 triples of errors, for trajecotry position error,
#         trajectory angular error, landmark position error, and
#         number of landmarks detected respectively.
#         Each triple is a pairwise comparison between estimate and truth,
#         toolbox EKF and truth, estimate and toolbox EKF respectively.

import numpy as np
import matplotlib.pyplot as plt
    
def visualize(x_est = None,P_est = None,indices = None,map = None,veh = None): 
    scipy.io.loadmat('e3.mat','map_s','veh_s')
    map = map_s
    veh = veh_s
    statistics = np.full([1,12],np.nan)
    # Plot ground truth landmarks (black hexagrams) and trajectory (blue line)
    figure
    scatter(map.map(1,:),map.map(2,:),'kh')
    hold('on')
    plt.plot(veh.x_hist(:,1),veh.x_hist(:,2),'b')
    # Visualize the estimated trajectory
    err_xtv,err_etv,err_xev,ang_xt,ang_et,ang_xe = visualize_path(x_est,P_est,veh,True)
    statistics[1,np.arange[1,6+1]] = np.array([err_xtv,err_etv,err_xev,ang_xt,ang_et,ang_xe])
    T = len(x_est)
    print('Trajectory errors (average Euclidean and angular distances over %d timesteps):\n' % (T))
    print('-- between estimate and ground truth: \n   Pos %f \tOri %f\n' % (err_xtv,ang_xt))
    # Visualize the estimated landmarks
    err_xtm,err_etm,err_xem,num_xt,num_et,num_xe = visualize_map(x_est,P_est,indices,map,True,True)
    statistics[1,np.arange[7,12+1]] = np.array([err_xtm,err_etm,err_xem,num_xt,num_et,num_xe])
    print('Landmark errors (average Euclidean distance for landmarks estimated by both):\n' % ())
    print('-- between estimate and ground truth (%d landmarks): \n   %f\n' % (num_xt,err_xtm))
    return statistics
    
    # Visualizes the estimated landmarks, ground truth,
# and optional toolbox EKF estimate (computed only if sol passed in).
# Returns error statistics between each pair of comparisons.
    
    # Input: x_est -> 1xT cell array containing the map state mean
#                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
#                 where M is the number of landmarks observed by time t)
#        P_est -> 1xT cell array containing the vehicle state covariance
#                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
#                 where M is the number of landmarks observed by time t)
#        indices -> Mx1 vector containing the landmark index corresponding
#                   to the entries in the state vector, where M is the
#                   number of landmarks observed by the final time step T)
#                   For example, if indices is [15; 4], then the first
#                   three rows of x_est and P_est correspond to the
#                   vehicle state, the next two correspond to landmark 15,
#                   and the next two rows correspond to landmark 4, etc.
#        map -> Robotics toolbox Map object containing the
#               known map (known landmarks)
#        is_slam -> Boolean value, true if in SLAM mode,
#                   false if in mapping mode with known vehicle poses
#        sol -> Robotics toolbox EKF object containing the toolbox estimate;
#               if passed in, the estimate will be visualized and errors
#               will be computed; if no solution, pass in empty matrix []
#        is_plot -> Boolean value; map is visualized if and only if true
#        compute_errors -> Boolean value; if true, errors are computed;
#                          if false, estimation errors are not computed
# Output: [err_xt, err_et, err_xe, num_xt, num_et, num_xe]
#         There are 2 triples of errors, for landmark position error
#         and number of landmarks detected respectively.
#         Each triple is a pairwise comparison between estimate and truth,
#         toolbox EKF and truth, estimate and toolbox EKF respectively.
    
    
def visualize_map(x_est = None,P_est = None,indices = None,map = None,is_slam = None,is_plot = None): 
    # If doing SLAM, need to offset indices by 3
# (first 3 elements of state vector are for vehicle position estimate)
    if is_slam:
        offset = 3
    else:
        offset = 0
    
    # Collect the estimated landmark positions
    T = len(x_est)
    M = (len(x_est[T]) - offset) / 2
    m_est = np.reshape(x_est[T](np.arange((1 + offset),end()+1)), tuple(np.array([2,M])), order="F")
    if is_plot:
        # Plot the estimated landmarks as red dots
        scatter(m_est(1,:),m_est(2,:),'r.')
        # Plot 95#-confidence ellipses around landmarks
        for i in np.arange(1,M+1).reshape(-1):
            plot_ellipse(P_est[T](np.arange((2 * i - 1 + offset),(2 * i + offset)+1),np.arange((2 * i - 1 + offset),(2 * i + offset)+1)) * 5.991,m_est(:,i),'r')
    
    # Compute errors between estimate, ground truth, and toolbox EKF estimate
# For each estimate, construct 2*M_truth matrix of landmark positions,
# where M_truth is the true number of landmarks (in argument map);
# undetected landmarks will have NaN values in their estimates
    map_t = map.map
    map_x = np.full([map_t.shape,map_t.shape],np.nan)
    for i in np.arange(1,len(indices)+1).reshape(-1):
        map_x[:,indices[i]] = x_est[T](np.arange((2 * i - 1 + offset),(2 * i + offset)+1))
    
    # Only compute errors between landmarks that are estimated by both
# Return number of landmarks that are estimated by both,
# as well as average error in the overlapping landmarks
    mask_xt = np.logical_and(not np.isnan(map_x) ,not np.isnan(map_t) )
    num_xt = sum(sum(mask_xt)) / 2
    err_xt = average_error(np.reshape(map_x(mask_xt), tuple(np.array([2,num_xt])), order="F"),np.reshape(map_t(mask_xt), tuple(np.array([2,num_xt])), order="F"))
    num_et = NaN
    err_et = NaN
    num_xe = NaN
    err_xe = NaN
    return err_xt,err_et,err_xe,num_xt,num_et,num_xe
    
    # Visualizes the estimated landmarks, ground truth,
# and optional toolbox EKF estimate (computed only if sol passed in).
# Returns error statistics between each pair of comparisons.
    
    # Input: x_est -> 1xT cell array containing the map state mean
#                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
#                 where M is the number of landmarks observed by time t)
#        P_est -> 1xT cell array containing the vehicle state covariance
#                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
#                 where M is the number of landmarks observed by time t)
#        veh -> Robotics toolbox Vehicle object containing the
#               known vehicle (known trajectory)
#        sol -> Robotics toolbox EKF object containing the toolbox estimate;
#               if passed in, the estimate will be visualized and errors
#               will be computed; if no solution, pass in empty matrix []
#        is_plot -> Boolean value; map is visualized if and only if true
#        compute_errors -> Boolean value; if true, errors are computed;
#                          if false, estimation errors are not computed
# Output: [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe];
#         There are 2 triples of errors, for trajecotry position error
#         and trajectory angular error respectively.
#         Each triple is a pairwise comparison between estimate and truth,
#         toolbox EKF and truth, estimate and toolbox EKF respectively.
    
    
def visualize_path(x_est = None,P_est = None,veh = None,is_plot = None): 
    # Collect the estimated vehicle trajectory
    T = len(x_est)
    v_est = np.zeros((T,3))
    for t in np.arange(1,T+1).reshape(-1):
        v_est[t,:] = x_est[t](np.arange(1,3+1))
    
    if is_plot:
        # Plot the estimated vehicle trajectory as a red line
        plt.plot(v_est(:,1),v_est(:,2),'r')
        # Plot 95#-confidence ellipses at every 10th state in trajectory
        for t in np.arange(1,T+10,10).reshape(-1):
            plot_ellipse(P_est[t](np.arange(1,2+1),np.arange(1,2+1)) * 5.991,v_est(t,np.arange(1,2+1)),'r')
    
    # Compute errors between estimate, ground truth, and toolbox EKF estimate
    err_xt = average_error(np.transpose(v_est(:,np.arange(1,2+1))),np.transpose(veh.x_hist(:,np.arange(1,2+1))))
    ang_xt = mean(np.abs(angdiff(v_est(:,3),veh.x_hist(:,3))))
    err_et = NaN
    err_xe = NaN
    ang_et = NaN
    ang_xe = NaN
    return err_xt,err_et,err_xe,ang_xt,ang_et,ang_xe
    
    # Computes the average Euclidean distance between A and B.
# Assumes both A and B are M*N matrices. Computes the Euclidean distance
# between respective column vectors in A and B (i.e., M-dim vectors),
# and returns the average over the N distances.
    
    
def average_error(A = None,B = None): 
    assert(np.all(A.shape == B.shape))
    err = np.average(np.sqrt(sum((A - B) ** 2)))
    return err