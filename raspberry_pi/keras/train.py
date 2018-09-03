import numpy as np
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

print("total data points: {}".format(dataset_total_length))
print("training data points: {}".format(dataset_train_length))

np.random.shuffle(dataset)

# define base model
def baseline_model():
  # create model
  model = Sequential()
  model.add(Dense(256, input_dim=4, kernel_initializer='normal', activation='relu'))
  model.add(Dense(1, kernel_initializer='normal'))
  # Compile model
  model.compile(loss='mean_squared_error', optimizer='adam')
  return model

train_data_x = dataset[:dataset_train_length,0:4]
train_data_y = dataset[:dataset_train_length,4]
validate_data_x = dataset[(dataset_train_length + 1):dataset_total_length,0:4]
validate_data_y = dataset[(dataset_train_length + 1):dataset_total_length,4] 


# fit(x=None, y=None, batch_size=None, epochs=1, verbose=1, 
# callbacks=None, validation_split=0.0, validation_data=None, 
# shuffle=True, class_weight=None, sample_weight=None, initial_epoch=0, 
# steps_per_epoch=None, validation_steps=None)


model = baseline_model()
model.fit(
  train_data_x, 
  train_data_y, 
  batch_size=32, 
  shuffle=True,
  epochs=1000,
  verbose=1, 
  validation_data=(validate_data_x, validate_data_y))

print("Saving model...")
model.save('model_h_256_b_32_e_1000.h5')

