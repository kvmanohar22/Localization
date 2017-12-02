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
   if args.filter == "Kalman":
      filter = K()
   elif args.filter == "Particle":
      filter = P()
   else:
      raise ValueError("No such filter is available!")
      return

   filter.simulate()
   filter.run()


if __name__=='__main__':
   main()