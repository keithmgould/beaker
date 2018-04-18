from arduino import Arduino

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def updateSetpoint(self, setpointString):
    self.arduino.updateSetpoint(setpointString[1:])

  def updatePid(self, pidString):
    if pidString == "Kx":
      self.arduino.updatePIDValues(0, 0, 0)
    else:
      pid = [float(x) for x in pidString[1:].split(",")]
      self.arduino.updatePIDValues(pid[0], pid[1], pid[2])


  def run(self):
    while True:
      print("Enter K<p,i,d> or Kx for 0,0,0")
      pidString = input()

      if pidString.startswith("K"):
        self.updatePid(pidString)
      elif pidString.startswith("S"):
        self.updateSetpoint(pidString)

agent = Agent()
agent.run()
