from lib.arduino import Arduino

# Use this little client to communicate with PID control, PID_ALL control, or P4 control

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def run(self):
    while True:
      print("Type H for help. Prefix a command with ! if not expecting a response.")
      userString = input()
      if userString[0] == '!':
      	userString = userString[1:]
      	self.arduino.writeMessage(userString)
      else:
      	print(self.arduino.writeMessageAndWait(userString))

agent = Agent()
agent.run()
