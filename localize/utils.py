"""
Contains all the utility functions
"""

import numpy as np
import matplotlib.pyplot as plt


def Gaussian(x, mu, sig):
   return np.array((1./(np.sqrt(2*np.pi)*sig))*(np.exp(-np.power(x-mu, 2)/(2.*np.power(sig, 2)))))


def add_gaussians(mu1, mu2, sig1, sig2):
   """
   Add two Gaussian distributions
   Args:
      mu1  : Mean of 1st distribution
      mu2  : Mean of 2nd distribution
      sig1 : Variance of 1st distribution
      sig1 : Variance of 2nd distribution
   
   Returns:
      mean, var
   """
   mu  = mu1+mu2
   sig_sq = np.power(sig1, 2)+np.power(sig2, 2)
   return mu, np.sqrt(sig_sq)


def mul_gaussians(mu1, mu2, sig1, sig2):
   """
   Multiply two Gaussian distributions
   Args:
      mu1  : Mean of 1st distribution
      mu2  : Mean of 2nd distribution
      sig1 : Variance of 1st distribution
      sig1 : Variance of 2nd distribution
   
   Returns:
      mean, var
   """
   sig1_sq = np.power(sig1, 2)
   sig2_sq = np.power(sig2, 2)
   mu = mu1 + (sig1_sq/(sig1_sq+sig2_sq))*(mu2-mu1)
   sig_sq = sig1_sq-(np.power(sig1_sq, 2) / (sig1_sq+sig2_sq))
   return mu, np.sqrt(sig_sq)