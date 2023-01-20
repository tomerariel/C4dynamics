import numpy as np

class proportional_navigation():    

    flyby = False
    rr = False
         
    def __init__(self, N, ts):
        self.N = N
        self.ts = ts

    def PN(self, v, lambda_dot):
        return self.N * v * lambda_dot
    
    def tgo():
        # TBD 
        pass 
    
    def set_flyby(self, rnew, r):
        
        rdot = (np.abs(rnew) - np.abs(r)) / self.ts

        if self.rr == False:
            if rdot < 0:
                self.rr = True
        elif rdot >= 0:
            self.flyby = True
                
                
                   