'''
Takes recorded data of the form:
s1a, s1b, s1c, s1d, a1, s2a, s2b, s2c, s2d

the above is a comma seperated line that will be used as follows:

s1 is an element from state 1
a1 is the action applied to s1
s2 is the resultant state.
there are four parameters to each state (a,b,c,d), so the entire row is 9 elements long.

We simply take this data and train a NN with supervised learning.
The input is s1a, s1b, s1c, s1d, a1
The expected output is s2a, s2b, s2c, s2d
'''

import numpy as np
import time
import sys
import pandas
import keras
from keras.models import Sequential
from keras.layers import Dense
from keras.models import load_model
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import pdb

if(len(sys.argv) != 3):
  print("give me <hidden size> <layers> please! like generate_blank_world_model 64 4")
  exit()

hidden_size = int(sys.argv[1])
layers = int(sys.argv[2])

print("hidden size: {}".format(hidden_size))
print("layers: {}".format(layers))

# input is 5 real numbers: 4 for the state and 5th for action
# output is 4 real numbers: the predicted next state
def baseline_model():
  model = Sequential()
  model.add(Dense(hidden_size, input_dim=5, kernel_initializer='normal', activation='relu'))
  for i in range(layers - 1):
  	model.add(Dense(hidden_size, kernel_initializer='normal', activation='relu'))
  model.add(Dense(4, kernel_initializer='normal'))
  model.compile(loss='mean_squared_error', optimizer='adam')
  return model

model = baseline_model()

modelName = "models/model_world_h{}x{}.h5".format(hidden_size, layers)
print("Saving model: {}".format(modelName))
model.save(modelName)

