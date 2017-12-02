import numpy as np
import time
import sys
import matplotlib.pyplot as plt

class Particle(object):
   """
   Implements Particle Filter (a Non Parametric Filter)
   """

   def __init__(self):
      self.X_0 = 0.1 # The initial true position
      self.X_prev = self.X_0
      self.X = self.X_0
      self.N = 100 # Number of particles
      self.T = 100 # Duration of observation
      self.R = 1 # Covariance of state update
      self.Q = 1 # Covariance of measurement
      self.R_0 = 2 # Variance of the initial estimate
      self.X_P = np.zeros(self.N).astype('float32') # Initial set of particles
      self.Xt =  np.zeros(self.N).astype('float32') # set of particles at any time t
      self.Zt =  np.zeros(self.N).astype('float32') # set of observations of particles at any time t
      self.W  =  np.zeros(self.N).astype('float32') # Weights of the particles (unnormalized)
      self.X_est  =  np.zeros(self.T).astype('float32') # Weights of the particles (unnormalized)


   def simulate(self):
      sys.stdout.write('Simulating the set of particles at t=0\n')
      for i in xrange(self.N):
         self.X_P[i] = self.X_0+np.sqrt(self.R_0)*(-.5+np.random.rand())

      # Visualize the initial set of particles
      plt.figure()
      plt.xlim([-5, 5])
      plt.xlabel('Time (sec)')
      plt.ylabel('Position (meters)')
      plt.title('Initial Particle distribution at t=0')
      plt.scatter(np.zeros(len(self.X_P)), self.X_P, c='k', s=9.9, label='Particles at t=0')
      plt.scatter(0, self.X, s=200, c='r', label='Initial True Position')
      plt.legend()
      plt.show()


   def run(self):
      f = lambda x, t : 0.5*x + (25*x)/(1+x**2) + 8*np.cos(1.2*(t-1)) + np.sqrt(self.R)*(-.5+np.random.rand()) 
      z = lambda x : (x**2)/20.#+np.sqrt(self.R)*(-.5+np.random.rand())
      w = lambda z, zt : (1./np.sqrt(2*np.pi*self.Q))*np.exp(-((z-zt)**2)/(2*self.Q))

      for t in xrange(self.T):
         sys.stdout.write('Simulating the particles at t=%d\n' % (t+1))
         # Actual Position of the robot
         self.X_prev = self.X
         self.X = f(self.X, t)
         # Measurement taken of the robot
         self.Z = z(self.X)

         for i in xrange(self.N):
            
            # Position of each particle conditioned on the position of the particles at `t-1`
            self.Xt[i] = f(self.X_P[i], t)

            # Measurement of each of the particles at time t based on the positions of the 
            # particles at t
            self.Zt[i] = z(self.Xt[i])

            # Weights of the particles
            self.W[i] = w(self.Z, self.Zt[i])

         # Normalize the weights
         self.W /= np.sum(self.W)

         sys.stdout.write('True measurement: %d\n' % (self.Z))
         sys.stdout.write('True position: %d\n' % (self.X))
         plt.subplot(121)
         plt.xlabel('Weight Magnitude: w[i]')
         plt.ylabel('Position values of particles: Xt[i]')
         plt.scatter(self.W, self.Xt, s=9.9, c='k', label='Particles at t=%d' % (t+1))
         plt.scatter(0, self.X, s=100, c='g', label='True Position')
         plt.title('Positional updates')
         plt.legend()
         plt.subplot(122)
         plt.xlabel('Weight Magnitude: w[i]')
         plt.ylabel('Measurement values: Zt[i]')
         plt.scatter(self.W, self.Zt, s=3, c='k', label='Measurement of particles at t=%d' % (t+1))
         plt.scatter(0, self.Z, s=100, c='r', label='True Measurement')
         plt.title('Measurement updates')
         plt.legend()
         plt.show()

         plt.subplot(131)
         plt.scatter(np.zeros(len(self.Xt)), self.Xt, s=3, c='k', label='Particles at t=dt-1')
         plt.scatter(0, self.X_prev, s=100, c='r', label='True Position at t=dt-1')
         plt.legend()

         # Resampling from the distribution
         for r in xrange(self.N):
            idx = np.array(np.nonzero(np.random.rand() <= np.cumsum(self.W)))[0][0]
            self.X_P[r] = self.Xt[idx]
         self.X_est[t] = np.mean(self.X_P)

         plt.subplot(132)
         plt.scatter(self.W, self.Xt, s=3, c='k') #label='Estimated position before resampling at t=dt')
         plt.scatter(0, self.X, s=100, c='g', label='True Position at t')
         plt.legend()

         plt.subplot(133)
         plt.scatter(np.zeros(len(self.X_P)), self.X_P, s=3, c='r', label='Particles at t=dt')
         plt.scatter(0, self.X_est[t], s=100, c='r', label='Estimated position at t')
         plt.scatter(0, self.X, s=100, c='g', label='True position at t')
         plt.legend()

         plt.show()