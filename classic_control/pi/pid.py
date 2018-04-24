from arduino import Arduino

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def updateSetpoint(self, setpointString):
    self.arduino.updateSetpoint(setpointString[1:])

  def updateThetaBias(self, biasString):
    self.arduino.updateThetaBias(biasString[1:])

  def updateMotorPid(self, pidString):
    if pidString == "Kx":
      self.arduino.updatePIDValues(0, 0, 0)
    else:
      pid = [float(x) for x in pidString[1:].split(",")]
      self.arduino.updatePIDValues(pid[0], pid[1], pid[2])

  def updateThetaPid(self, pidString):
      pid = [float(x) for x in pidString[1:].split(",")]
      self.arduino.updateThetaPIDValues(pid[0], pid[1], pid[2])


  def run(self):
    while True:
      print("Enter K<p,i,d> or Kx for 0,0,0, S<setpoint> for new Motor Setpoint, or P<p,i,d> for theta PID")
      pidString = input()

      if pidString.startswith("K"):
        self.updateMotorPid(pidString)
      elif pidString.startswith("S"):
        self.updateSetpoint(pidString)
      elif pidString.startswith("P"):
        self.updateThetaPid(pidString)
      elif pidString.startswith("B"):
        self.updateThetaBias(pidString)


agent = Agent()
agent.run()
