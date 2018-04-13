from arduino import Arduino
import argparse


class Agent:
  def __init__(self, args):
    self.args = args
    self.arduino = Arduino()

  def run(self):
    a = 1


parser = argparse.ArgumentParser(description='provide arguments for agent')
parser.add_argument('--p', help='P value', default=0)
parser.add_argument('--i', help='I value', default=0)
parser.add_argument('--d', help='D value', default=0)
args = vars(parser.parse_args())

agent = Agent(args)
agent.run()

