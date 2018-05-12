from arduino import Arduino

# Use this little client to communicate with PID control, PID_ALL control, or P4 control

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def run(self):
    while True:
      print("Type H for help.")
      userString = input()
      self.arduino.writeMessageAndWait(userString)

agent = Agent()
agent.run()
