import numpy as np
import time
import sys
import pandas
from keras.models import Sequential
from keras.layers import Dense
from keras.models import load_model
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import pdb


if(len(sys.argv) != 2):
  print("give me a log file as argument please!")
  exit()

logfile = sys.argv[1]
# load dataset
dataframe = pandas.read_csv(logfile, header=None)
dataset = dataframe.values
dataset_total_length = len(dataset)
dataset_train_length = int(dataset_total_length * 0.8)

np.random.shuffle(dataset)

hidden_size = 256
batch_size = 32
epochs = 2000

print("total data points: {}".format(dataset_total_length))
print("training data points: {}".format(dataset_train_length))
print("hidden size: {}".format(hidden_size))
print("batch size: {}".format(batch_size))
print("epochs: {}".format(epochs))

# define base model
def baseline_model():
  # create model
  model = Sequential()
  model.add(Dense(hidden_size, input_dim=4, kernel_initializer='normal', activation='relu'))
  model.add(Dense(1, kernel_initializer='normal'))
  # Compile model
  model.compile(loss='mean_squared_error', optimizer='adam')
  return model

train_data_x = dataset[:dataset_train_length,0:4]
train_data_y = dataset[:dataset_train_length,4]
validate_data_x = dataset[(dataset_train_length + 1):dataset_total_length,0:4]
validate_data_y = dataset[(dataset_train_length + 1):dataset_total_length,4] 

model = baseline_model()
history = model.fit(
  train_data_x, 
  train_data_y, 
  batch_size=32, 
  shuffle=True,
  epochs=epochs,
  verbose=1, 
  validation_data=(validate_data_x, validate_data_y))

timestr = time.strftime("%Y%m%d-%H%M%S")
modelName = "model_dataset_length_{}_h_".format(dataset_total_length) + str(hidden_size) +"_b_" + str(batch_size) + "_e_" + str(epochs)  + "_" + timestr +  "_from_logfile_" + logfile + "_val_loss_" + "{0:.3f}".format(history.history['val_loss'][-1])  + ".h5"
print("Saving model: {}".format(modelName))
model.save(modelName)

