# Input: odo -> 2xT matrix containing odometry readings for T time steps
#        zind -> 1xT vector containing the observed landmark index for
#                T time steps; index is 0 if no landmark observed
#        z -> 1xT cell array containing the (range, bearing) observation
#             for T time steps; z{t} is empty if no observation at time t
#        V -> 2x2 matrix denoting the process noise in (forward, angular)
#        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
#        x0 -> 3x1 vector denoting the initial vehicle state mean
#        P0 -> 3x3 matrix denoting the initial vehicle state covariance
# Output: x_est -> 1xT cell array containing the map state mean
#                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
#                  where M is the number of landmarks observed by time t)
#         P_est -> 1xT cell array containing the vehicle state covariance
#                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
#                  where M is the number of landmarks observed by time t)
#         indices -> Mx1 vector containing the landmark index corresponding
#                    to the entries in the state vector, where M is the
#                    number of landmarks observed by the final time step T)
#                    For example, if indices is [15; 4], then the first
#                    three rows of x_est and P_est correspond to the
#                    vehicle state, the next two correspond to landmark 15,
#                    and the next two rows correspond to landmark 4, etc.

import numpy as np
from math import fabs, pi
    
def E3(odo = None,zind = None,z = None,V = None,W = None,x0 = None,P0 = None): 
    time = len(odo[0])
    x_est = np.array([x0])
    P_est = np.array([P0])
    seenLandmarks = np.array([]).astype(int)

    for t in range(time):
        # ----------------- prediction step of EKF -----------------
        # predicted mean
        x_pred = convertPos(odo[:,t],x_est[t])
        theta = x_est[t][2]
        # predicted covariance
        Fx = calculateTransitionJacobian(odo[0,t],theta,seenLandmarks)
        Fv = calcTransitionVarJacob(theta,seenLandmarks)
        pPred = Fx @ P_est[t] @ np.transpose(Fx) + Fv @ V @ np.transpose(Fv)
        landmarkId = zind[t].astype(int)
        # ----------------- decide what action to perform -----------------
        if landmarkId == 0:
            # predict next position and covariance based off of odometry
            x_est = np.concatenate((x_est, [x_pred]), axis=0)
            P_est = np.concatenate((P_est, [pPred]), axis=0)
        else:
            if landmarkId in seenLandmarks:
                # update with Hx and joint motion model
                Hx = createJointJacobian(x_est[t],theta,z[t],seenLandmarks,landmarkId)
                error = calculateError(x_est[t],landmarkId,seenLandmarks,z[t],theta)
                K = pPred * np.transpose(Hx) * (Hx * pPred * np.transpose(Hx) + W) ** - 1
                x_est[t + 1] = x_pred + K * error
                x_est[t + 1][3] = wrapToPi(x_est[t + 1](3))
                P_est[t + 1] = pPred - K * Hx * pPred
            else:
                # insert new landmark
                seenLandmarks = np.append(seenLandmarks, landmarkId)
                # calculate insertion jacobian
                Yz = createInsertionJacobian(theta,z[t],pPred)
                # predict next position based on odometry and add the new
                # landmark pos
                x_est[t + 1] = np.array([[x_pred],[convertLandmarkPos(x_est[t],z[t],theta)]])
                # add landmark to covariance matrix with insertion matrix
                pSize = pPred.shape
                P_est[t + 1] = Yz * np.array([[pPred,np.zeros((pSize(1),2))],[np.zeros((2,pSize(2))),W]]) * np.transpose(Yz)
    
    indices = np.transpose(seenLandmarks)
    return x_est,P_est,indices
    
    # convert odometry readings to a new world coordinate
    
def convertPos(odometry = None,oldRobotState = None): 
    deltaD = odometry[0]
    deltaTheta = odometry[1]
    pos = oldRobotState.astype(float)
    pos[0] = oldRobotState[0] + deltaD * np.cos(oldRobotState[2])
    pos[1] = oldRobotState[1] + deltaD * np.sin(oldRobotState[2])
    pos[2] = wrapToPi(oldRobotState[2] + deltaTheta)
    return pos
    
    # create the insertion jacobian Yz
    
def createInsertionJacobian(theta = None,measurment = None,pPred = None): 
    beta = measurment(2)
    r = measurment(1)
    Gx = np.array([[1,0,- r * np.sin(wrapToPi(theta + beta))],
                   [0,1,r * np.cos(wrapToPi(theta + beta))]])
    Gz = np.array([[np.cos(wrapToPi(theta + beta)),
                    - r * np.sin(wrapToPi(theta + beta))],
                    [np.sin(wrapToPi(theta + beta)),
                     r * np.cos(wrapToPi(theta + beta))]])
    n = pPred.shape[1-1]
    YzCell = np.array([])
    YzCell[1,1] = np.eye(n,n)
    YzCell[1,2] = np.zeros((n,2))
    YzCell[2,1] = Gx
    YzCell[2,2] = np.zeros((2,n - 3))
    YzCell[2,3] = Gz
    if pPred.shape[1-1] == 0:
        Yz = np.array([Gx,Gz])
    else:
        Yz = YzCell
    
    return Yz
    
    # create the joint motion model Hx
    
def createJointJacobian(robotState = None,theta = None,measurement = None,seenLandmarks = None,landmarkId = None): 
    landmarkPos = convertLandmarkPos(robotState,measurement,theta)
    # current robot position
    x_v = robotState[0]
    y_v = robotState[1]
    # measured landmark pose
    r = measurement[0]
    x_i = landmarkPos[0]
    y_i = landmarkPos[1]
    Hpi = np.array([[(x_i - x_v) / r, (y_i - y_v) / r],
                    [- (y_i - y_v) / r ** 2, (x_i - x_v) / r ** 2]])
    # est landmark position
    index = np.where(seenLandmarks == landmarkId)
    Hx_v = np.array([[- (x_i - x_v) / r, -(y_i - y_v) / r,0],
                     [(y_i - y_v) / r ** 2, - (x_i - x_v) / r ** 2,- 1]])
    # construct Hx matrix
    sizeLandmarks = seenLandmarks.shape[2-1]
    zeros1 = np.zeros((2,2 * (index - 1)))
    zeros2 = np.zeros((2,2 * (sizeLandmarks - index)))
    Hx = np.array([Hx_v,zeros1,Hpi,zeros2])
    return Hx
    
    # convert a landmark measurment to a world position
    
def convertLandmarkPos(robotState = None,measurement = None,theta = None): 
    landmarkPos = np.zeros((2,1))
    r = measurement(1)
    beta = measurement(2)
    landmarkPos[1] = robotState(1) + r * np.cos(wrapToPi(theta + beta))
    landmarkPos[2] = robotState(2) + r * np.sin(wrapToPi(theta + beta))
    return landmarkPos
    
    # calculate the error of the measurement
    
def calculateError(robotState = None,landmarkId = None,seenLandmarks = None,measurement = None,theta = None): 
    # get previous est of landmark pose
    index = seenLandmarks.toList().index(landmarkId)
    x_i = robotState(3 + 2 * index - 1)
    y_i = robotState(3 + 2 * index)
    # get current estimate of robot pose
    x_v = robotState(1)
    y_v = robotState(2)
    # calculate the angle of the measurement (not bearing) and convert
# from tan domain
    tanMeasurement = np.arctan((y_i - y_v) / (x_i - x_v))
    if (x_i - x_v) < 0:
        tanMeasurement = tanMeasurement + np.pi
    
    # predicted measurment based on current robot estimate and
# known landmark pose
    measurementPred = np.array([[np.sqrt((y_i - y_v) ** 2 + (x_i - x_v) ** 2)],[wrapToPi(tanMeasurement - theta)]])
    error = measurement - measurementPred
    error[2] = wrapToPi(error(2))
    return error
    
    # calculate the transition function jacobian for the prediced covariance
# matrix
    
def calculateTransitionJacobian(delta = None,theta = None,seenLandmarks = None): 
    FxBase = np.array([[1,0, -delta * np.sin(theta)],
                       [0,1, delta * np.cos(theta)],
                       [0,0,1]])
    numLandmarks = len(seenLandmarks)
    Fx = np.concatenate((FxBase, np.zeros((3,2 * numLandmarks))), axis=1)
    Fx = np.concatenate((Fx, np.concatenate((np.zeros((2 * numLandmarks, 3)), np.eye(2 * numLandmarks, 2 * numLandmarks)), axis=1)), axis=0)
    # Fx = np.array([[FxBase, np.zeros((3,2 * numLandmarks))],
    #                [np.zeros((2 * numLandmarks, 3)), np.eye(2 * numLandmarks, 2 * numLandmarks)]])
    return Fx
    
    # calculate the transition variance jacobian for the predicted covariance
# matrix
    
def calcTransitionVarJacob(theta = None,seenLandmarks = None): 
    FvBase = np.array([[np.cos(theta),0],
                       [np.sin(theta),0],
                       [0,1]])
    numLandmarks = len(seenLandmarks)
    Fv = np.concatenate((FvBase, np.zeros((2 * numLandmarks,2))), axis=0)
    return Fv
    
def wrapToPi(input_angle):
    revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))

    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(fabs((truncated_remainder(input_angle + pi, 2 * pi))
                                      / (2 * pi))) - 1))) * pi

    output_angle = p1 - p2

    return output_angle

def truncated_remainder(dividend, divisor):
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)

    remainder = dividend - divisor * divided_number

    return remainder