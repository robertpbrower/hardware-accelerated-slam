% Visualizes the estimated trajectory and landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).

% Input: x_est -> 1xT cell array containing the map state mean
%                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                 where M is the number of landmarks observed by time t)
%        P_est -> 1xT cell array containing the vehicle state covariance
%                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                 where M is the number of landmarks observed by time t)
%        indices -> Mx1 vector containing the landmark index corresponding
%                   to the entries in the state vector, where M is the
%                   number of landmarks observed by the final time step T)
%                   For example, if indices is [15; 4], then the first
%                   three rows of x_est and P_est correspond to the
%                   vehicle state, the next two correspond to landmark 15,
%                   and the next two rows correspond to landmark 4, etc.
%        map -> Robotics toolbox Map object containing the
%               known map (known landmarks)
%        veh -> Robotics toolbox Vehicle object containing the
%               known vehicle (known trajectory)
%        mode -> 'l' for localization, 'm' for mapping, 's' for SLAM,
%                'h' for statistics
%                In 'l', 'm', 's' modes, the ground truth and estimated
%                trajectory and/or map is visualized, and estimation errors
%                are printed out if compute_errors is true
%                In 'h' mode, visualization and printing is suppressed,
%                since many trials are assumed.
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
%                          (unless in 'h' mode); this allows partial
%                          trajectories and maps to be visualized as well.
% Output: statistics -> 1x12 vector of error statistics
%         = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe,
%            err_xtm, err_etm, err_xem, num_xt, num_et, num_xe]
%         There are 4 triples of errors, for trajecotry position error,
%         trajectory angular error, landmark position error, and
%         number of landmarks detected respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function statistics = visualize(x_est, P_est, indices)

    load("e3_new.mat", 'map', 'x_hist');

    statistics = NaN(1, 12);

    % Plot ground truth landmarks (black hexagrams) and trajectory (blue line)
    figure;
    scatter(map(1,:), map(2,:), 'kh');
    hold on;
    plot(x_hist(:,1), x_hist(:,2), 'b');

    % Visualize the estimated trajectory
    [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, x_hist, true);

    statistics(1, 1:6) = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe];
    

    T = length(x_est);
    fprintf('Trajectory errors (average Euclidean and angular distances over %d timesteps):\n', T);
    fprintf('-- between estimate and ground truth: \n   Pos %f \tOri %f\n', err_xtv, ang_xt);

    % Visualize the estimated landmarks
    [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, true, true);

    statistics(1, 7:12) = [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe];
      
    fprintf('Landmark errors (average Euclidean distance for landmarks estimated by both):\n');
    fprintf('-- between estimate and ground truth (%d landmarks): \n   %f\n', num_xt, err_xtm);

end




% Visualizes the estimated landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Returns error statistics between each pair of comparisons.

% Input: x_est -> 1xT cell array containing the map state mean
%                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                 where M is the number of landmarks observed by time t)
%        P_est -> 1xT cell array containing the vehicle state covariance
%                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                 where M is the number of landmarks observed by time t)
%        indices -> Mx1 vector containing the landmark index corresponding
%                   to the entries in the state vector, where M is the
%                   number of landmarks observed by the final time step T)
%                   For example, if indices is [15; 4], then the first
%                   three rows of x_est and P_est correspond to the
%                   vehicle state, the next two correspond to landmark 15,
%                   and the next two rows correspond to landmark 4, etc.
%        map -> Robotics toolbox Map object containing the
%               known map (known landmarks)
%        is_slam -> Boolean value, true if in SLAM mode,
%                   false if in mapping mode with known vehicle poses
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        is_plot -> Boolean value; map is visualized if and only if true
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
% Output: [err_xt, err_et, err_xe, num_xt, num_et, num_xe]
%         There are 2 triples of errors, for landmark position error
%         and number of landmarks detected respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function [err_xt, err_et, err_xe, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, is_slam, is_plot)
    % If doing SLAM, need to offset indices by 3
    % (first 3 elements of state vector are for vehicle position estimate)
    if is_slam
        offset = 3;
    else
        offset = 0;
    end

    % Collect the estimated landmark positions
    T = length(x_est);
    M = (length(x_est{T}) - offset) / 2;
    m_est = reshape(x_est{T}((1+offset):end), [2 M]);
    if is_plot
        % Plot the estimated landmarks as red dots
        scatter(m_est(1,:), m_est(2,:), 'r.');
        % Plot 95%-confidence ellipses around landmarks
        for i = 1:M
            plot_ellipse(P_est{T}((2*i-1+offset):(2*i+offset),(2*i-1+offset):(2*i+offset)) * 5.991, m_est(:,i), 'r');
        end
    end
    
    
 
    % Compute errors between estimate, ground truth, and toolbox EKF estimate
    % For each estimate, construct 2*M_truth matrix of landmark positions,
    % where M_truth is the true number of landmarks (in argument map);
    % undetected landmarks will have NaN values in their estimates
    map_t = map;
    map_x = NaN(size(map_t));
    for i = 1:length(indices)
        map_x(:,indices(i)) = x_est{T}((2*i-1+offset):(2*i+offset));
    end

    % Only compute errors between landmarks that are estimated by both
    % Return number of landmarks that are estimated by both,
    % as well as average error in the overlapping landmarks
    mask_xt = ~isnan(map_x) & ~isnan(map_t);
    num_xt = sum(sum(mask_xt)) / 2;  
    err_xt = average_error(reshape(map_x(mask_xt), [2 num_xt]), reshape(map_t(mask_xt), [2 num_xt]));
    num_et = NaN;
    err_et = NaN;
    num_xe = NaN;
    err_xe = NaN;
        
end

% Visualizes the estimated landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Returns error statistics between each pair of comparisons.

% Input: x_est -> 1xT cell array containing the map state mean
%                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                 where M is the number of landmarks observed by time t)
%        P_est -> 1xT cell array containing the vehicle state covariance
%                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                 where M is the number of landmarks observed by time t)
%        veh -> Robotics toolbox Vehicle object containing the
%               known vehicle (known trajectory)
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        is_plot -> Boolean value; map is visualized if and only if true
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
% Output: [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe];
%         There are 2 triples of errors, for trajecotry position error
%         and trajectory angular error respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, x_hist, is_plot)

    % Collect the estimated vehicle trajectory
    T = length(x_est);
    v_est = zeros(T,3);
    for t = 1:T
        v_est(t,:) = x_est{t}(1:3);
    end
    
    if is_plot
        % Plot the estimated vehicle trajectory as a red line
        plot(v_est(:,1), v_est(:,2), 'r');
        % Plot 95%-confidence ellipses at every 10th state in trajectory
        for t = 1:10:T
            plot_ellipse(P_est{t}(1:2,1:2) * 5.991, v_est(t,1:2), 'r');
        end
    end
    
    % Compute errors between estimate, ground truth, and toolbox EKF estimate
    err_xt = average_error(v_est(:,1:2)', x_hist(:,1:2)');
    ang_xt = mean(abs(angdiff(v_est(:,3), x_hist(:,3))));

      
   


    err_et = NaN;
    err_xe = NaN;
    ang_et = NaN;
    ang_xe = NaN;
        
end


% Computes the average Euclidean distance between A and B.
% Assumes both A and B are M*N matrices. Computes the Euclidean distance
% between respective column vectors inS A and B (i.e., M-dim vectors),
% and returns the average over the N distances.

function err = average_error(A, B)
    assert(all(size(A) == size(B)));
    err = mean(sqrt(sum((A - B).^2)));
end

function value = angdiff(th1, th2)
     value = mod((th1 - th2)+pi, 2*pi) - pi;
end

function handles = plot_ellipse(E, varargin)
    
    assert(size(E,1) == size(E,2), 'ellipse is defined by a square matrix');
    assert( size(E,1) == 2 || size(E,1) == 3, 'can only plot ellipsoid for 2 or 3 dimenions');

    npoints = 40;
    arglist = varargin(1);

    % process the probability
    s = 1;
    
    % ellipse centre is provided
    centre = arglist{1};
   
    holdon = ishold();
    hold on

    % plot an ellipse
    
    % define points on a unit circle
    th = linspace(0, 2*pi, npoints);
    pc = [cos(th);sin(th)];
    
    % warp it into the ellipse
    pe = sqrtm(E)*pc * s;
    
    % offset it to optional non-zero centre point
    centre = centre(:);
    if nargin > 1
        pe = bsxfun(@plus, centre(1:2), pe);
    end
    x = pe(1,:); y = pe(2,:);

    % plot 2D data
    z = zeros(size(x));
    
    h = plot3(x', y', z', 'r');

  if ~holdon
      hold off
  end
    
    if nargout > 0
        handles = h;
    end
end