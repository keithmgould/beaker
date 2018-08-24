from lib.arduino import Arduino
from keras.models import load_model
import numpy as np
import time
import pdb

def main():
	arduino = Arduino()
	model = load_model("./model_256_b_32_e_1000.h5")
	obs = arduino.waitForState()

	while(True):
		foo = np.array(obs)
		bar = foo.reshape((1, -1))
		action = model.predict(bar, batch_size=1)
		arduino.updateMotorPower(action)

if __name__ == '__main__':
  main()