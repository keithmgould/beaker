from arduino import Arduino

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def run(self):
    while True:
      print("Enter Command Type followed by Command. Ex: P1 2 3")
      userString = input()

      if(userString == 'H'):
        self.arduino.getPidValues()
      else:
        self.arduino.writeMessage(userString)

agent = Agent()
agent.run()
