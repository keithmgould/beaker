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

hidden_size = 64


print("hidden size: {}".format(hidden_size))

# input is 5 real numbers: 4 for the state and 5th for action
# output is 4 real numbers: the predicted next state
def baseline_model():
  model = Sequential()
  model.add(Dense(hidden_size, input_dim=5, kernel_initializer='normal', activation='relu'))
  model.add(Dense(hidden_size, kernel_initializer='normal', activation='relu'))
  model.add(Dense(hidden_size, kernel_initializer='normal', activation='relu'))
  model.add(Dense(4, kernel_initializer='normal'))
  # Compile model
  model.compile(loss='mean_squared_error', optimizer='adam')
  return model

model = baseline_model()

timestr = time.strftime("%Y%m%d-%H%M%S")
modelName = "model_world_h{}.h5".format(hidden_size)
print("Saving model: {}".format(modelName))
model.save(modelName)

