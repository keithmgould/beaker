import numpy as np
import sys
import pandas

if(len(sys.argv) != 2):
  print("give me a log file as argument please!")
  exit()

logfile = sys.argv[1]
# load dataset
dataframe = pandas.read_csv(logfile, header=None)
dataset = dataframe.values

# every thousand steps, the env resets, and the state jumps to almost all zeros.
# We want to be careful that we don't include this data. So first some data massaging.
# We will make the input and output all on the same row, and ignore the appropriate values:

# I'm sure there is a neato numpy way to do this :/
wideDataset = np.array([0,0,0,0,0,0,0,0,0])
for idx, row in enumerate(dataset):
  if(row[1] == 0.0 and row[2] == 0.0 and row[3] == 0.0):
    print("skipping row {}: {}".format(idx, row))
    continue

  newRow = np.concatenate([dataset[idx-1][0:5],row[0:4]])
  wideDataset = np.vstack([wideDataset, newRow])

wideDataframe = pandas.DataFrame(wideDataset)
wideDataframe.to_csv("wide_version_of_" + logfile, header = False, index = False)