import argparse

from localize import Kalman as K
from localize import Histogram as H
from localize import Particle as P
from localize import Markov as M

parser = argparse.ArgumentParser(description='Localization for Mobile Robots in Continuous Space')
parser.add_argument('--filter', help='The filter that you want to run', default='Kalman', type=str,
   choices=['Kalman', 'Histogram', 'Markov', 'Particle'])

args = parser.parse_args()
def main():
   k = K()
   k.simulate()
   k.kalman()


if __name__=='__main__':
   main()