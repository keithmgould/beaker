import numpy as np
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

# load dataset
dataframe = pandas.read_csv("./pid_log_virtual_70K_rows-20180817.txt", header=None)
dataset = dataframe.values

# shuffle order of input/output pairs
np.random.shuffle(dataset)

# split into input (X) and output (Y) variables
X = dataset[:50000,0:4]
Y = dataset[:50000,4]

pdb.set_trace()

# define base model
def baseline_model():
	# create model
	model = Sequential()
	model.add(Dense(128, input_dim=4, kernel_initializer='normal', activation='relu'))
	model.add(Dense(1, kernel_initializer='normal'))
	# Compile model
	model.compile(loss='mean_squared_error', optimizer='adam')
	return model

# fix random seed for reproducibility
seed = 7
np.random.seed(seed)
# evaluate model with standardized dataset
estimator = KerasRegressor(build_fn=baseline_model, epochs=1000, batch_size=100, verbose=1)
estimator.fit(X,Y)

# kfold = KFold(n_splits=10, random_state=seed)
# results = cross_val_score(estimator, X, Y, cv=kfold)
# print("Results: %.2f (%.2f) MSE" % (results.mean(), results.std()))

print("Saving model...")
estimator.model.save('model4.h5')

