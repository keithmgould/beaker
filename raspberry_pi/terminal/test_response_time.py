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
      observation = self.arduino.writeMessageAndWait("S") # get State
      print("obs: {}".format(observation[3]))
      wheelSpeed = float(observation[3])
      if(wheelSpeed > 0.0):
        responseTime = current_milli_time() - issueTime
        print("response time: {}".format(responseTime))
        return
      else:
        print("{},{}".format(wheelSpeed, observation))


  def ensureStopped(self):
    print("Ensuring stopped...")
    while True:
      observation = self.arduino.writeMessageAndWait("S") # get State
      print(observation)
      wheelSpeed = float(observation[3])
      if(wheelSpeed == 0.0):
        print("Stopped! :)")
        return
      else:
        print("wheel speed is: {}".format(wheelSpeed))
        self.arduino.writeMessage("W0")
        time.sleep(0.5)

agent = Agent()
agent.run()
