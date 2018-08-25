from lib.arduino import Arduino
from keras.models import load_model
import numpy as np
import time
import pdb

def main():
  arduino = Arduino()
  model = load_model("./model_h_256_b_32_e_1000.h5")
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
