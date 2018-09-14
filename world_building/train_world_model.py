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
from loss_charter import LossCharter
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
  print("give me a world-model and a log file as arguments please!")
  exit()

world_model_file = sys.argv[1]

#python is the worst
world_model_file_no_extension = "".join(world_model_file.split(".")[0:-1])

log_file = sys.argv[2]
log_code = log_file.split(".")[0]


# load world model
world_model = load_model(world_model_file)

# load dataset
dataframe = pandas.read_csv(log_file, header=None)
dataset = dataframe.values
dataset_total_length = len(dataset)
dataset_train_length = int(dataset_total_length * 0.8)

np.random.shuffle(dataset)

batch_size = 32
epochs = 10

print("total data points: {}".format(dataset_total_length))
print("training data points: {}".format(dataset_train_length))
print("batch size: {}".format(batch_size))
print("epochs: {}".format(epochs))

train_data_x = dataset[:dataset_train_length,0:5]
train_data_y = dataset[:dataset_train_length,5:9]
validate_data_x = dataset[(dataset_train_length + 1):dataset_total_length,0:5]
validate_data_y = dataset[(dataset_train_length + 1):dataset_total_length,5:9]

history = world_model.fit(
  train_data_x, 
  train_data_y, 
  batch_size=32, 
  shuffle=True,
  epochs=epochs,
  verbose=1, 
  validation_data=(validate_data_x, validate_data_y))

loss = history.history['val_loss'][-1]
model_name = "{}.{}_b{}_e{}".format(world_model_file_no_extension,log_code, batch_size, epochs) + "_{0:3f}_.h5".format(loss)

lc = LossCharter()
lc.chart(history.history, model_name)

print("Saving model: {}".format(model_name))
world_model.save(model_name)

