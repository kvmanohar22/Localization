import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt
from numpy.linalg import inv
from scipy.signal import savgol_filter

import sys

from ..utils import Gaussian

class Kalman(object):
   """
   Implements Kalman Filter (a Parametric Filter)
   """

   def __init__(self):
      sys.stdout.write('Starting Kalman Filter...\n')
      self.duration = 10
      self.dt = 0.1

      # Matrices for calculating the following equations
      '''
      x_bar_t = A_t * x_{t-1} + B_t * u_t + R_t
      z_bar_t = C_t * x_bar_t + Q_t
      '''
      self.A_t = np.array([[1, self.dt], [0, 1]])
      self.B_t = np.array([np.power(self.dt, 2), self.dt]).T
      self.C_t = np.array([1, 0]).T

      self.u_t = 1.5 # Control input (acceleration in m/s^2) given at any time `t`
      self.X_t = np.array([0, 0]).T # [position, velocity].T
      self.X_bar_t = self.X_t
      self.u_noise = 9 # std dev of acceleration (m/s^2)
      self.z_noise = 10   # std dev of location (m)
      self.Q_t = np.power(self.z_noise, 2) # Covariance matrix of the observation (position only)
      self.R_t = np.power(self.u_noise, 2) * np.array(
         [[(self.dt**4)/4., (self.dt**3)/2.],
          [(self.dt**3)/2., (self.dt**2)]]) # Covariance matrix of the model
      self.Sigma_t = self.R_t

      self.X_act = np.array([]) # Actual position of the motion
      self.V_act = np.array([]) # Actual velocity of the motion
      self.X_msr = np.array([]) # Position as observed by the observer (z_t)

   def simulate(self):
      print 'Simulating the motion of the qual...'
      plt.figure()
      plt.xlim([0, 120])
      plt.ylim([-5, 90])
      plt.title('Qual tracking by Ninja')
      plt.xlabel('Time (sec)')
      plt.ylabel('X position (meter)')
      dt = self.dt
      _range = np.arange(0, self.duration, dt)
      for i in _range:
         # Actual flight of motion with added Gaussian noise
         epsilon_t = self.u_noise * np.array([(np.random.rand()-0.5)*(dt**2)/2., (np.random.rand()-0.5)*dt]).T
         self.X_t = np.matmul(self.A_t, self.X_t) + self.B_t * self.u_t + epsilon_t

         # This is the sensor data as would be seen by the observer with added noise
         delta_t = self.z_noise * (np.random.rand()-0.5)
         z = np.matmul(self.C_t, self.X_t) + delta_t

         self.X_act = np.append(self.X_act, self.X_t[0])
         self.X_msr = np.append(self.X_msr, z)
         self.V_act = np.append(self.V_act, self.X_t[1])

         new_range = np.arange(0, len(self.X_act))
         plt.plot(new_range, self.X_act, marker='*', c='r', ms=.5, label='Actual position')
         plt.plot(new_range, self.X_msr, marker='o', c='k', ms=.5, label='Measured position')
         plt.pause(0.0001)

   def kalman(self):
      print 'Implementing Kalman filter...'
      plt.figure()
      plt.xlim([0, 120])
      plt.ylim([-5, 90])
      plt.title('Qual tracking by Ninja using Kalman Filter')
      plt.xlabel('Time (sec)')
      plt.ylabel('X position (meter)')

      X_bar = np.array([])
      X = np.array([])
      sig = np.array([])
      sig_bar = np.array([])
      for i in range(len(self.X_act)):
         self.X_bar_t = np.matmul(self.A_t, self.X_bar_t) + self.B_t * self.u_t
         self.Sigma_t = np.matmul(np.matmul(self.A_t, self.Sigma_t), self.A_t.T)+self.R_t
         X_bar = np.append(X_bar, self.X_bar_t[0])
         sig_bar = np.append(sig_bar, self.Sigma_t[0][0])

         Kalman_gain  = np.matmul(self.Sigma_t, self.C_t.T) * (1. / (
            np.matmul(np.matmul(self.C_t, self.Sigma_t), self.C_t.T)+self.Q_t))

         self.X_bar_t = self.X_bar_t + Kalman_gain * (self.X_msr[i]-np.matmul(self.C_t, self.X_bar_t))
         self.Sigma_t = np.matmul(np.eye(2)- Kalman_gain * self.C_t, self.Sigma_t)
         X = np.append(X, self.X_bar_t[0])
         sig = np.append(sig, self.Sigma_t[0][0])

      mean_pos = savgol_filter(self.X_msr, 9, 3)
      plt.plot(range(0, self.X_msr.shape[0]), self.X_msr, marker='o', c='r', ms=.5, label='Observeration')
      plt.plot(range(0, self.X_act.shape[0]), self.X_act, marker='*', c='b', ms=.5, label='Actual Path')
      plt.plot(range(0, len(mean_pos)), mean_pos, marker='o', c='k', ms=.5, label='Smoothened (Savitzky Golay Filter)')
      plt.plot(range(0, X.shape[0]), X, marker='*', c='g', ms=.5, label='Kalman Tracking')
      plt.legend()

      # Plotting the distributions
      plt.figure()
      plt.title('Distribution comparison')
      for i in xrange(len(self.X_act)):

         # Actual Position of the quail
         x_act = self.X_act[i]
         plt.axvline(x_act, c='k', label='True position')

         pos = np.arange(-X[i]-15, X[i]+15, 0.01)

         # Predicted distribution
         mean = X_bar[i]
         std = sig_bar[i]
         y = Gaussian(pos, mean, std)
         y = y / np.max(y)
         plt.plot(pos, y, c='r', label='Predictd before measurement')

         # Measurement distribution
         mean = self.X_msr[i]
         std = self.z_noise
         y = Gaussian(pos, mean, std)
         y = y / np.max(y)
         plt.plot(pos, y, c='b', label='Measurement distribution')

         # Combined distribution of the position of the quail
         mean = X[i]
         std  = sig[i]
         y = Gaussian(pos, mean, std)
         y = y / np.max(y)
         plt.plot(pos, y, c='g', label='Combined distribution')
         plt.legend(loc='lower center', fancybox=False, shadow=True, ncol=2)
         plt.savefig('{}'.format(i))
         plt.pause(0.05)
         plt.clf()
         plt.cla()
