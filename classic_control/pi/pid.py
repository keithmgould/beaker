from arduino import Arduino

class Agent:
  def __init__(self):
    self.arduino = Arduino()

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

  def updateControlValues(self, userString):
      values = [float(x) for x in userString[1:].split(",")]
      self.arduino.updateThetavaluesValues(values[0], values[1], values[2], values[3])


  def run(self):
    while True:
      print("Enter K<p,i,d> or Kx for 0,0,0, S<setpoint> for new Motor Setpoint, or P<p,i,d> for theta PID")
      userString = input()

      if userString.startswith("K"):
        self.updateMotorPid(userString)
      elif userString.startswith("P"):
        self.updateThetaPid(userString)
      elif userString.startswith("S"):
        self.arduino.updateSetpoint(userString[1:])
      elif userString.startswith("B"):
        self.arduino.updateThetaBias(userString[1:])
      elif userString.startswith("C"):
        self.updateControlValues(userString)

agent = Agent()
agent.run()
