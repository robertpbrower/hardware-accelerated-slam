% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first
%                    three rows of x_est and P_est correspond to the
%                    vehicle state, the next two correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E3(odo, zind, z, V, W, x0, P0)
    time = size(odo,2);
    x_est = {x0};
    P_est = {P0};
    seenLandmarks = {};

    for t = 1:time - 1
        % ----------------- prediction step of EKF -----------------
        % predicted mean
        x_pred = convertPos(odo(:,t), x_est{t});
        theta = x_est{t}(3);
        % predicted covariance
        Fx = calculateTransitionJacobian(odo(1,t), theta, seenLandmarks);
        Fv = calcTransitionVarJacob(theta, seenLandmarks);
        pPred = Fx*P_est{t}*transpose(Fx) + Fv*V*transpose(Fv);
        landmarkId = zind(t);

        % ----------------- decide what action to perform -----------------

        if landmarkId == 0
            % predict next position and covariance based off of odometry
            x_est{t+1} = x_pred;
            P_est{t+1} = pPred;
        elseif ismember(landmarkId, cell2mat(seenLandmarks))
            % update with Hx and joint motion model
            Hx = createJointJacobian(x_est{t}, theta, z{t}, seenLandmarks, landmarkId);
            error = calculateError(x_est{t}, landmarkId, seenLandmarks, z{t}, theta);
            K = pPred*transpose(Hx)*(Hx*pPred*transpose(Hx) + W)^-1;
            x_est{t+1} = x_pred + K*error;
            x_est{t+1}(3) = wrapToPi(x_est{t+1}(3));
            P_est{t+1} = pPred - K*Hx*pPred;
        else
            % insert new landmark

            seenLandmarks{size(seenLandmarks, 2) + 1} = landmarkId;
            % calculate insertion jacobian
            Yz = createInsertionJacobian(theta, z{t}, pPred);
            % predict next position based on odometry and add the new
            % landmark pos
            x_est{t+1} = [x_pred; convertLandmarkPos(x_est{t}, z{t}, theta)];
            % add landmark to covariance matrix with insertion matrix
            pSize = size(pPred);
            P_est{t+1} = Yz*[pPred zeros(pSize(1), 2);
                             zeros(2, pSize(2)) W]*transpose(Yz);
          

        end
    end
    indices = transpose(cell2mat(seenLandmarks));

end

% convert odometry readings to a new world coordinate
function pos = convertPos(odometry, oldRobotState)
    deltaD = odometry(1);
    deltaTheta = odometry(2);
    pos = oldRobotState;
    pos(1) = oldRobotState(1) + deltaD*cos(oldRobotState(3));
    pos(2) = oldRobotState(2) + deltaD*sin(oldRobotState(3));
    pos(3) = wrapToPi(oldRobotState(3) + deltaTheta);
end

% create the insertion jacobian Yz
function Yz = createInsertionJacobian(theta, measurment, pPred)
    beta = measurment(2);
    r = measurment(1);
    Gx = [1 0 -r*sin(wrapToPi(theta + beta));
          0 1 r*cos(wrapToPi(theta + beta))];
    Gz = [cos(wrapToPi(theta + beta)) -r*sin(wrapToPi(theta + beta));
          sin(wrapToPi(theta + beta)) r*cos(wrapToPi(theta + beta))];
    n = size(pPred, 1);

    YzCell = {};
    YzCell{1,1} = eye(n,n);
    YzCell{1,2} = zeros(n,2);
    YzCell{2,1} = Gx;
    YzCell{2,2} = zeros(2, n-3);
    YzCell{2,3} = Gz;
    if size(pPred, 1) == 0
        Yz = [Gx Gz];
    else
        Yz = cell2mat(YzCell);
    end
end

% create the joint motion model Hx
function Hx = createJointJacobian(robotState, theta, measurement, seenLandmarks, landmarkId)
    landmarkPos = convertLandmarkPos(robotState, measurement, theta);
    % current robot position
    x_v = robotState(1);
    y_v = robotState(2);

    % measured landmark pose
    r = measurement(1);
    x_i = landmarkPos(1);
    y_i = landmarkPos(2);
    Hpi = [(x_i-x_v)/r   (y_i-y_v)/r;
           -(y_i-y_v)/r^2 (x_i-x_v)/r^2];

    % est landmark position
    index = find(cell2mat(seenLandmarks) == landmarkId);
    Hx_v = [-(x_i-x_v)/r   -(y_i-y_v)/r   0;
            (y_i-y_v)/r^2 -(x_i-x_v)/r^2 -1];
    
    % construct Hx matrix
    sizeLandmarks = size(seenLandmarks, 2);
    zeros1 = zeros(2, 2*(index-1));
    zeros2 = zeros(2, 2*(sizeLandmarks-index));
    Hx = [Hx_v zeros1 Hpi zeros2];
end

% convert a landmark measurment to a world position
function landmarkPos = convertLandmarkPos(robotState, measurement, theta)
    landmarkPos = zeros(2, 1);
    r = measurement(1);
    beta = measurement(2);
    landmarkPos(1) = robotState(1) + r*cos(wrapToPi(theta + beta));
    landmarkPos(2) = robotState(2) + r*sin(wrapToPi(theta + beta));
end

% calculate the error of the measurement
function error = calculateError(robotState, landmarkId, seenLandmarks, measurement, theta)
    % get previous est of landmark pose
    index = find(cell2mat(seenLandmarks) == landmarkId);
    x_i = robotState(3 + 2*index - 1);
    y_i = robotState(3 + 2*index);
    % get current estimate of robot pose 
    x_v = robotState(1);
    y_v = robotState(2);

    % calculate the angle of the measurement (not bearing) and convert
    % from tan domain
    tanMeasurement = atan((y_i-y_v)/(x_i-x_v));
    if (x_i - x_v) < 0
        tanMeasurement = tanMeasurement + pi;
    end
    % predicted measurment based on current robot estimate and
    % known landmark pose
    measurementPred = [sqrt((y_i-y_v)^2 + (x_i-x_v)^2);
                       wrapToPi(tanMeasurement - theta)];
    error = measurement - measurementPred;
    error(2) = wrapToPi(error(2));
end

% calculate the transition function jacobian for the prediced covariance
% matrix
function Fx = calculateTransitionJacobian(delta, theta, seenLandmarks)
    FxBase = [1 0 -delta*sin(theta);
              0 1 delta*cos(theta);
              0 0 1];
    numLandmarks = size(seenLandmarks, 2);
    Fx = [FxBase zeros(3, 2*numLandmarks);
          zeros(2*numLandmarks, 3) eye(2*numLandmarks, 2*numLandmarks)];
end

% calculate the transition variance jacobian for the predicted covariance
% matrix
function Fv = calcTransitionVarJacob(theta, seenLandmarks)
    FvBase = [cos(theta) 0;
               sin(theta) 0;
               0 1];
    numLandmarks = size(seenLandmarks, 2);
    Fv = [FvBase; zeros(2*numLandmarks, 2)];
end