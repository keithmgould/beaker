from arduino import Arduino

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def run(self):
    while True:
      print("Enter p,i,d   or x for 0,0,0")
      pidString = input()
      if pidString == "x":
        print("stopping!")
        pid = [0,0,0]
      else:
        pid = [float(x) for x in pidString.split(",")]
      self.arduino.updatePIDValues(pid[0], pid[1], pid[2])

agent = Agent()
agent.run()
