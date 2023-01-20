import numpy as np

class luenberger:
  """ 
  luenberger estimator 
  agranovich, 
  modern control 72
  """

  A = 0
  c = 0
  obsv = 0  
  Aest = 0

  def __init__(self, A, c): 
    self.A = A
    self.c = c
    self.obsv = np.copy(self.c)
    for n in range(len(self.A) - 1):
      self.obsv = np.vstack((self.obsv, self.c @ self.A**(n + 1))).copy()
   
  def isobservable(self):
    return np.linalg.matrix_rank(self.obsv) == len(self.A)
        
  def eig(self):
    return np.linalg.eig(self.A)[0]
  
  def setest(self, s):
    n = len(self.A)

    # coefficients from the desired eigenvalues
    an_d = np.polynomial.polynomial.polyfromroots(s)
    # the extended system eigenvalues are including the luenberger gains which are currently unknown.
    # the polynomial that represents the system is given by the determinant of s*I-Aest. where Aest is the extended system matrix which inclueds the gains.
    # the desired eigenvalues are given in the input argument s. 
    # assuming the prime coefficient is one in the both systems, there are n-1 coefficients to compare.


    #
    # the calculation of luenberger gains
    #   rational:
    #   1 calculate the luenberger gains of the equivalent canonical model
    #   2 find the model transform matrix 
    #   3 tranform the gains 
    ##

    # a canonical equivalent system matrix:
    #   | 0 0 ..  -a0 |
    #   | 1 0 ..  -a1 |
    #   | ..          |
    #   |0 0 .. 1 -an-1|
    #   where a0..an-1 are the coefficients of the system polynomial.

    an = np.polynomial.polynomial.polyfromroots(np.linalg.eig(self.A)[0])
    # canA = np.zeros((n, n))
    # for i in range(n):
    #   canA[i, -1] = -an[i]
    #   if i == 0:
    #     continue
    #   canA[i, i - 1] = 1

    # luenberger gains for the canonical system
    Lc = np.zeros(n) # Lcanonical 
    for i in range(n):
      Lc[i] = an_d[i] - an[i]

    # model transformation matrix
    cmu = np.zeros(n)
    cmu[-1] = 1
    mu = np.linalg.solve(self.obsv, cmu.reshape((-1, 1)))

    M = np.copy(mu)
    for n in range(n - 1):
      M = np.hstack((M, self.A**(n + 1) @ mu)).copy()

    #   3 tranform the gains 
    L = M @ Lc

    self.Aest = self.A - L @ self.c
    
  