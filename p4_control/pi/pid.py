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
      self.arduino.updateControlValues(values[0], values[1], values[2], values[3])


  def run(self):
    while True:
      print("Enter Command Type followed by Command. Ex: P1 2 3")
      userString = input()

      self.arduino.

agent = Agent()
agent.run()
