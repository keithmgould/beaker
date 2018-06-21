from lib.arduino import Arduino
import time

# Use this script to test how quickly we can tell stopped wheels to start spinning and see that in an observation

# utility for testing loop times
current_milli_time = lambda: int(round(time.time() * 1000))

class Agent:
  def __init__(self):
    self.arduino = Arduino()

  def run(self):
    self.ensureStopped()
    issueTime = current_milli_time()
    self.arduino.writeMessage("W10")

    while True:
      observation = self.arduino.getState()
      wheelSpeed = float(observation[3])
      if(wheelSpeed > 0.0):
        responseTime = current_milli_time() - issueTime
        print("response time: {}".format(responseTime))
        break

    self.ensureStopped()
    print("End of experiment!")

  def ensureStopped(self):
    print("Ensuring stopped...")
    self.arduino.writeMessage("W0")
    while True:
      observation = self.arduino.getState()
      wheelSpeed = float(observation[3])
      if(wheelSpeed == 0.0):
        print("Stopped! :)")
        return
      else:
        time.sleep(0.5)

agent = Agent()
agent.run()
