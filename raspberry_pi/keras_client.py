from lib.arduino import Arduino
from keras.models import load_model
import numpy as np
import time
import pdb

def main():

  if(len(sys.argv) != 2):
    print("give me a model file as argument please!")
    exit()

  modelfile = sys.argv[1]

  arduino = Arduino()
  model = load_model(modelfile)
  print("waiting for first state...")
  obs = arduino.waitForState()
  print("neat found first state!")

  while(True):
    obs = arduino.waitForState()
    foo = np.array(obs)
    bar = foo.reshape((1, -1))
    action = model.predict(bar, batch_size=1)
    action = action[0][0]
    print(action)
    arduino.updateMotorPower(action)

if __name__ == '__main__':
  main()
