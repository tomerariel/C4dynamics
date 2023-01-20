import numpy as np

class lowpass:
  """ 
  low pass filter
  zarchan noise analysis
  """

  tau = 0 # filter time constant 
  ts = 0  # integration step size
  x = 0
  
  def __init__(self, tau, ts, x): 
    self.tau = tau
    self.ts = ts
    self.x = np.reshape(x, ((3, 1)))
      
  def predict(self):
    pass
    
  def update(self, f, xin):
    dx = -self.x[0] / self.tau + xin / self.tau
    self.x[0] = self.x[0] + dx * self.ts
    return self.x
    