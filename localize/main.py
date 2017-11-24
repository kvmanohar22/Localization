import argparse

import Kalman as K
import Histogram as H
import Particle as P
import Markov as M

parser = argparse.ArgumentParser(description='Localization for Mobile Robots in Continuous Space')
parser.add_argument('--filter', help='The filter that you want to run', default='Kalman', type=str,
   choices=['Kalman', 'Histogram', 'Markov', 'Particle'])

args = parser.parse_args()
def main():
   pass