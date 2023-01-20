import numpy as np

class e_kalman:
  """ 
  extended kalman filter 
  zarchan 374 
  see also   
  https://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf#:~:text=A%20Kalman%20Filtering%20is%20carried,in%20wireless%20networks%20is%20given.
  
  """
  x = 0 # state vector. 
  # Phi = 0 # transition matrix 
  P = 0   # covariance matrix
  # Q = 0   # process noise matrix
  H = 0   # measurement matrix 
  R = 0   # measurement noise matrix 
  
  tau = 0

  def __init__(self, x0, p0noise, tau):  # vp, 
    '''    '''
    
    self.x = np.reshape(x0, ((3, 1)))

    n = len(x0)

    # obj.Phi = np.zeros(n)
    self.P = np.zeros((n, n))
    for i in range(n):
      # the variance of the error in the initial estimate of position and is taken to be the variance of the measurement noise.
      # the variance of the error in the initial estimate of velocity
      # the variance of the error in the initial estimate of ballistic coefficient
      # others: assumed that there is no process noise.
      self.P[i, i] = p0noise[i]**2
    # obj.Q = np.zeros(n)         

    self.H = np.zeros((n))
    self.H[0] = 1
    self.R = p0noise[0]**2
    self.tau = tau 
    
      
      
  def predict(self, f, Phi, Q):
    '''
    predict the mean X and the covariance P of the system state at time k.
    x input mean state estimate of the previous step (k - 1)
    P state covariance at k - 1
    A transition matrix
    Q process noise covariance matrix
    b input matrix 
    u control input 
    '''
    self.x = self.x + f(self.x) * self.tau
    self.P = np.linalg.multi_dot([Phi, self.P, Phi.T]) + Q   

    return self.x
 
 
  def update(self, f, y_in):
    '''
    computes the posterior mean x and covariance P of the state given new measurement y.
    corrects x and P given the predicted x and P matrices, measurement vector y, the measurement 
    matrix H and the measurement covariance matrix R
    K kalman gains
    '''
    
    S = self.R + np.linalg.multi_dot([self.H, self.P, self.H.T])

    invs = 1 / S if S.ndim < 2 else np.linalg.inv(S)
    K = np.reshape(np.dot(self.P @ self.H.T, invs), (len(self.P), -1))

    self.x = self.x + np.dot(K, y_in - np.dot(self.H, self.x)).reshape((len(K), 1))
    self.P = (np.eye(len(self.P)) - K * self.H) @ self.P

    return self.x

